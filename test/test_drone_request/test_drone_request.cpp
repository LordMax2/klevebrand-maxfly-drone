#include <math.h>
#include <stdio.h>
#include <string.h>

#include "drone_request.h"

static int failures = 0;

static void expect_true(const bool condition, const char *name)
{
    if (!condition)
    {
        printf("FAIL %s\n", name);
        failures++;

        return;
    }

    printf("PASS %s\n", name);
}

static void expect_float(const float actual, const float expected, const char *name)
{
    const float delta = actual - expected;
    const float abs_delta = fabsf(delta);

    expect_true(abs_delta < 0.0001f, name);
}

static void test_five_field_csv_omits_altitude()
{
    char payload[] = "2,1,1,18.0686,59.3293";
    const DroneRequest_t request = DroneRequest_t::parseFromCsvString(payload);

    expect_true(request.flight_mode_id == 2, "five-field flight_mode_id");
    expect_true(request.enable_power, "five-field enable_power");
    expect_true(request.enable_motors, "five-field enable_motors");
    expect_float(request.longitude, 18.0686f, "five-field longitude");
    expect_float(request.latitude, 59.3293f, "five-field latitude");
    expect_float(request.assignment_altitude, 0.0f, "five-field assignment_altitude is 0");
    expect_true(!request.hasCommandedAltitude(), "five-field has no commanded altitude");
}

static void test_six_field_csv_parses_altitude()
{
    char payload[] = "1,0,1,18.0686,59.3293,42.5";
    const DroneRequest_t request = DroneRequest_t::parseFromCsvString(payload);

    expect_true(request.flight_mode_id == 1, "six-field flight_mode_id");
    expect_true(!request.enable_power, "six-field enable_power");
    expect_true(request.enable_motors, "six-field enable_motors");
    expect_float(request.assignment_altitude, 42.5f, "six-field assignment_altitude");
    expect_true(request.hasCommandedAltitude(), "six-field has commanded altitude");
}

static void test_six_field_csv_zero_altitude_is_not_commanded()
{
    char payload[] = "1,1,1,18.0686,59.3293,0";
    const DroneRequest_t request = DroneRequest_t::parseFromCsvString(payload);

    expect_float(request.assignment_altitude, 0.0f, "zero assignment_altitude");
    expect_true(!request.hasCommandedAltitude(), "zero altitude is not a command");
}

static void test_six_field_csv_blank_altitude_is_not_commanded()
{
    char payload[] = "1,1,1,18.0686,59.3293,";
    const DroneRequest_t request = DroneRequest_t::parseFromCsvString(payload);

    expect_float(request.assignment_altitude, 0.0f, "blank field 6 assignment_altitude");
    expect_true(!request.hasCommandedAltitude(), "blank field 6 is not a command");
}

static void test_json_parses_assignment_altitude()
{
    char payload[] =
        "{\"flightModeId\":2,\"enablePower\":true,\"enableMotors\":false,"
        "\"lat\":59.3293,\"lon\":18.0686,\"assignmentAltitude\":25}";
    const DroneRequest_t request = DroneRequest_t::parseFromCsvString(payload);

    expect_true(request.flight_mode_id == 2, "json flight_mode_id");
    expect_true(request.enable_power, "json enable_power");
    expect_true(!request.enable_motors, "json enable_motors");
    expect_float(request.latitude, 59.3293f, "json latitude");
    expect_float(request.longitude, 18.0686f, "json longitude");
    expect_float(request.assignment_altitude, 25.0f, "json assignmentAltitude");
    expect_true(request.hasCommandedAltitude(), "json has commanded altitude");
}

static void test_json_omitted_altitude_is_not_commanded()
{
    char payload[] = "{\"flightModeId\":1,\"enableMotors\":true,\"lat\":59.3,\"lon\":18.0}";
    const DroneRequest_t request = DroneRequest_t::parseFromCsvString(payload);

    expect_float(request.assignment_altitude, 0.0f, "json omitted assignmentAltitude");
    expect_true(!request.hasCommandedAltitude(), "json omitted altitude is not a command");
}

static void test_json_zero_altitude_is_not_commanded()
{
    char payload[] = "{\"assignmentAltitude\":0}";
    const DroneRequest_t request = DroneRequest_t::parseFromCsvString(payload);

    expect_float(request.assignment_altitude, 0.0f, "json zero assignmentAltitude");
    expect_true(!request.hasCommandedAltitude(), "json zero altitude is not a command");
}

static void test_json_pascal_case_assignment_altitude()
{
    char payload[] = "{\"AssignmentAltitude\":12.25}";
    const DroneRequest_t request = DroneRequest_t::parseFromCsvString(payload);

    expect_float(request.assignment_altitude, 12.25f, "json AssignmentAltitude");
    expect_true(request.hasCommandedAltitude(), "json PascalCase has commanded altitude");
}

static void test_empty_request_has_no_commanded_altitude()
{
    const DroneRequest_t request = DroneRequest_t::empty();

    expect_float(request.assignment_altitude, 0.0f, "empty assignment_altitude");
    expect_true(!request.hasCommandedAltitude(), "empty request is not a command");
}

int main()
{
    test_five_field_csv_omits_altitude();
    test_six_field_csv_parses_altitude();
    test_six_field_csv_zero_altitude_is_not_commanded();
    test_six_field_csv_blank_altitude_is_not_commanded();
    test_json_parses_assignment_altitude();
    test_json_omitted_altitude_is_not_commanded();
    test_json_zero_altitude_is_not_commanded();
    test_json_pascal_case_assignment_altitude();
    test_empty_request_has_no_commanded_altitude();

    if (failures != 0)
    {
        printf("%d test(s) failed\n", failures);

        return 1;
    }

    printf("all tests passed\n");

    return 0;
}
