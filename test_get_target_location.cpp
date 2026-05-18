// Test for GetTargetLocation() — runs standalone without PSDK.
//
// Build:
//   g++ -std=c++11 -o test_get_target_location test_get_target_location.cpp
//
// Unit tests (default):
//   ./test_get_target_location
//   ./test_get_target_location --test
//
// Live polling (run alongside target_location_publisher.py):
//   ./test_get_target_location --live
//   ./test_get_target_location --live --hz 5

#include <cstdio>
#include <ctime>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <unistd.h>     // unlink, usleep
#include <csignal>

// ---- Mock DJI types & macros ------------------------------------------------
static constexpr double DJI_PI = 3.14159265358979323846;

struct T_DjiFlightControllerHomeLocation {
    double latitude;
    double longitude;
};

#define USER_LOG_INFO(fmt, ...)  printf("[INFO]  " fmt "\n", ##__VA_ARGS__)
#define USER_LOG_WARN(fmt, ...)  printf("[WARN]  " fmt "\n", ##__VA_ARGS__)
#define USER_LOG_ERROR(fmt, ...) printf("[ERROR] " fmt "\n", ##__VA_ARGS__)

// ---- Copy of GetTargetLocation (must match main.cpp) ------------------------
static constexpr const char *TARGET_FILE   = "/tmp/rth_target.txt";
static constexpr double      MAX_STALE_SEC = 5.0;

static bool GetTargetLocation(T_DjiFlightControllerHomeLocation &out) {
    FILE *f = fopen(TARGET_FILE, "r");
    if (!f) {
        USER_LOG_ERROR("Target file not found: %s", TARGET_FILE);
        return false;
    }

    double lat_deg, lon_deg, ts;
    int n = fscanf(f, "%lf,%lf,%lf", &lat_deg, &lon_deg, &ts);
    fclose(f);

    if (n != 3) {
        USER_LOG_ERROR("Target file parse error (expected 3 fields, got %d).", n);
        return false;
    }

    double age_sec = difftime(time(nullptr), static_cast<time_t>(ts));
    if (age_sec > MAX_STALE_SEC) {
        USER_LOG_WARN("Target data stale (%.1f s old, max %.0f s). Is publisher running?",
                      age_sec, MAX_STALE_SEC);
        return false;
    }

    out.latitude  = lat_deg * DJI_PI / 180.0;
    out.longitude = lon_deg * DJI_PI / 180.0;
    return true;
}

// ---- Test helpers -----------------------------------------------------------
static int s_pass = 0;
static int s_fail = 0;

static void check(const char *name, bool condition) {
    if (condition) {
        printf("    PASS  %s\n", name);
        s_pass++;
    } else {
        printf("    FAIL  %s  <<<\n", name);
        s_fail++;
    }
}

static void write_file(const char *content) {
    FILE *f = fopen(TARGET_FILE, "w");
    fprintf(f, "%s", content);
    fclose(f);
}

static bool approx(double a, double b, double eps = 1e-9) {
    return std::fabs(a - b) < eps;
}

// ---- Test cases -------------------------------------------------------------
static void test_file_not_found() {
    printf("\n[1] file not found\n");
    unlink(TARGET_FILE);

    T_DjiFlightControllerHomeLocation loc;
    check("returns false", !GetTargetLocation(loc));
}

static void test_malformed_two_fields() {
    printf("\n[2] malformed — only 2 fields\n");
    write_file("12.994319,101.443273\n");

    T_DjiFlightControllerHomeLocation loc;
    check("returns false", !GetTargetLocation(loc));
    unlink(TARGET_FILE);
}

static void test_malformed_garbage() {
    printf("\n[3] malformed — garbage text\n");
    write_file("hello world\n");

    T_DjiFlightControllerHomeLocation loc;
    check("returns false", !GetTargetLocation(loc));
    unlink(TARGET_FILE);
}

static void test_stale_data() {
    printf("\n[4] stale timestamp (10 s ago)\n");
    char buf[128];
    snprintf(buf, sizeof(buf), "12.994319,101.443273,%.3f\n",
             static_cast<double>(time(nullptr) - 10));
    write_file(buf);

    T_DjiFlightControllerHomeLocation loc;
    check("returns false", !GetTargetLocation(loc));
    unlink(TARGET_FILE);
}

static void test_boundary_inside() {
    printf("\n[5] timestamp just inside limit (age = MAX_STALE_SEC - 1)\n");
    char buf[128];
    snprintf(buf, sizeof(buf), "12.994319,101.443273,%.3f\n",
             static_cast<double>(time(nullptr) - static_cast<time_t>(MAX_STALE_SEC) + 1));
    write_file(buf);

    T_DjiFlightControllerHomeLocation loc;
    check("returns true", GetTargetLocation(loc));
    unlink(TARGET_FILE);
}

static void test_boundary_outside() {
    printf("\n[6] timestamp just outside limit (age = MAX_STALE_SEC + 1)\n");
    char buf[128];
    snprintf(buf, sizeof(buf), "12.994319,101.443273,%.3f\n",
             static_cast<double>(time(nullptr) - static_cast<time_t>(MAX_STALE_SEC) - 1));
    write_file(buf);

    T_DjiFlightControllerHomeLocation loc;
    check("returns false", !GetTargetLocation(loc));
    unlink(TARGET_FILE);
}

static void test_valid_conversion() {
    printf("\n[7] valid fresh data — correct radian conversion\n");
    const double LAT_DEG = 12.994319;
    const double LON_DEG = 101.443273;

    char buf[128];
    snprintf(buf, sizeof(buf), "%.8f,%.8f,%.3f\n",
             LAT_DEG, LON_DEG, static_cast<double>(time(nullptr)));
    write_file(buf);

    T_DjiFlightControllerHomeLocation loc = {0.0, 0.0};
    check("returns true", GetTargetLocation(loc));
    check("latitude  = deg * PI / 180", approx(loc.latitude,  LAT_DEG * DJI_PI / 180.0));
    check("longitude = deg * PI / 180", approx(loc.longitude, LON_DEG * DJI_PI / 180.0));
    unlink(TARGET_FILE);
}

static void test_negative_coords() {
    printf("\n[8] negative coordinates (southern / western hemisphere)\n");
    const double LAT_DEG = -33.868820;   // Sydney
    const double LON_DEG = 151.209296;

    char buf[128];
    snprintf(buf, sizeof(buf), "%.8f,%.8f,%.3f\n",
             LAT_DEG, LON_DEG, static_cast<double>(time(nullptr)));
    write_file(buf);

    T_DjiFlightControllerHomeLocation loc = {0.0, 0.0};
    check("returns true",              GetTargetLocation(loc));
    check("latitude negative in rad",  loc.latitude < 0.0);
    check("latitude correct",          approx(loc.latitude,  LAT_DEG * DJI_PI / 180.0));
    check("longitude correct",         approx(loc.longitude, LON_DEG * DJI_PI / 180.0));
    unlink(TARGET_FILE);
}

// ---- Live polling mode ------------------------------------------------------
static volatile bool s_live_running = true;
static void on_live_signal(int) { s_live_running = false; }

static void run_live(double hz) {
    signal(SIGINT,  on_live_signal);
    signal(SIGTERM, on_live_signal);

    unsigned int period_us = static_cast<unsigned int>(1e6 / hz);
    printf("Live polling at %.1f Hz  (Ctrl+C to stop)\n", hz);
    printf("FILE : %s\n", TARGET_FILE);
    printf("%-12s  %-14s  %-14s  %s\n", "time", "lat (deg)", "lon (deg)", "status");
    printf("%-12s  %-14s  %-14s  %s\n",
           "------------", "--------------", "--------------", "-------");

    while (s_live_running) {
        T_DjiFlightControllerHomeLocation loc = {0.0, 0.0};
        bool ok = GetTargetLocation(loc);

        char ts_buf[16];
        time_t now = time(nullptr);
        strftime(ts_buf, sizeof(ts_buf), "%H:%M:%S", localtime(&now));

        if (ok) {
            double lat_deg = loc.latitude  * 180.0 / DJI_PI;
            double lon_deg = loc.longitude * 180.0 / DJI_PI;
            printf("%-12s  %14.6f  %14.6f  OK\n", ts_buf, lat_deg, lon_deg);
        } else {
            printf("%-12s  %-14s  %-14s  NO DATA\n", ts_buf, "--", "--");
        }
        fflush(stdout);
        usleep(period_us);
    }
    printf("\nStopped.\n");
}

// ---- Unit tests entry -------------------------------------------------------
static void run_tests() {
    printf("=== GetTargetLocation unit tests ===\n");
    printf("    TARGET_FILE   : %s\n", TARGET_FILE);
    printf("    MAX_STALE_SEC : %.0f\n", MAX_STALE_SEC);

    test_file_not_found();
    test_malformed_two_fields();
    test_malformed_garbage();
    test_stale_data();
    test_boundary_inside();
    test_boundary_outside();
    test_valid_conversion();
    test_negative_coords();

    printf("\n=== Results: %d passed, %d failed ===\n", s_pass, s_fail);
}

// ---- Entry point ------------------------------------------------------------
int main(int argc, char **argv) {
    bool live_mode = false;
    double hz = 1.0;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--live") == 0) {
            live_mode = true;
        } else if (strcmp(argv[i], "--test") == 0) {
            live_mode = false;
        } else if (strcmp(argv[i], "--hz") == 0 && i + 1 < argc) {
            hz = atof(argv[++i]);
            if (hz <= 0.0) { fprintf(stderr, "ERROR: --hz must be > 0\n"); return 1; }
        } else {
            fprintf(stderr, "Usage: %s [--live] [--test] [--hz <rate>]\n", argv[0]);
            return 1;
        }
    }

    if (live_mode) {
        run_live(hz);
        return 0;
    }

    run_tests();
    return s_fail > 0 ? 1 : 0;
}
