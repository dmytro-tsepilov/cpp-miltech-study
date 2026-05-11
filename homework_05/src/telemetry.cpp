#include "telemetry.hpp"

#include <format>
#include <cstdlib>
#include <fstream>
#include <iostream>

// Debugging exercise notes:
// this file intentionally contains four runtime defects.
// The defects are related to malformed input shape, invalid numeric values,
// unsafe time deltas, and empty logs. Exact locations are not marked on purpose.

const int EXPECTED_FIELD_COUNT = 7;
const int MAX_LINE_LENGTH = 256;

Frame parse_frame(char line[], int &error_code, std::string &error_message);
bool is_valid_frame(const int &line_number, const Frame &current_frame, const Frame &previous_frame = {});

int split_line(char line[], char* fields[], int max_fields) {
    int count = 0;
    char* cursor = line;

    while (*cursor != '\0' && count < max_fields) {
        while (*cursor == ' ' || *cursor == '\t' || *cursor == '\n' || *cursor == '\r') {
            *cursor = '\0';
            ++cursor;
        }

        if (*cursor == '\0') {
            break;
        }

        fields[count] = cursor;
        ++count;

        while (*cursor != '\0' && *cursor != ' ' && *cursor != '\t' && *cursor != '\n' &&
               *cursor != '\r') {
            ++cursor;
        }
    }

    return count;
}

long parse_long(const char* text, int &error_code) {
    char* end = nullptr;
    const long value = std::strtol(text, &end, 10);

    if (end == text) {
        std::cerr << __FUNCTION__ << "-> error: invalid value: " << text << std::endl;
        error_code = 3;
        return -1;
    }

    return value;
}

int parse_int(const char* text, int &error_code) {
    return static_cast<int>(parse_long(text, error_code));
}

double parse_double(const char* text, int &error_code) {
    char* end = nullptr;
    const double value = std::strtod(text, &end);

    if (end == text) {
        std::cerr << __FUNCTION__ << "-> error: invalid value: " << text << std::endl;
        error_code = 3;
        return -1;
    }

    return value;
}

Frame parse_frame(char line[], int &error_code, std::string &error_message) {
    char* fields[EXPECTED_FIELD_COUNT] = {};
    const int field_count = split_line(line, fields, EXPECTED_FIELD_COUNT);

    if (field_count != EXPECTED_FIELD_COUNT) {
        error_message = std::format("error: expected {} fields, but got {}", EXPECTED_FIELD_COUNT, field_count);
        error_code = 1;
        return {};
    }

    Frame frame{};
    frame.timestamp_ms = parse_long(fields[0], error_code);
    frame.seq = parse_int(fields[1], error_code);
    frame.voltage_v = parse_double(fields[2], error_code);
    frame.current_a = parse_double(fields[3], error_code);
    frame.temperature_c = parse_double(fields[4], error_code);
    frame.gps_fix = parse_int(fields[5], error_code);
    frame.satellites = parse_int(fields[6], error_code);

    if (error_code != 0) {
        error_message = std::format("{} -> error ->", __FUNCTION__);
        return {};
    }

    return frame;
}

bool is_valid_frame(const int &line_number, const Frame &current_frame, const Frame &previous_frame) {

    // voltage_v > 0;
    if (current_frame.voltage_v <= 0) {
        std::cerr << __FUNCTION__ << "-> parse error line: " << line_number << " -> voltage_v: <= 0" << std::endl;
        return false;
    }

    // temperature_c у діапазоні [-40, 120];
    if (current_frame.temperature_c > 120 || current_frame.temperature_c < -40) {
        std::cerr << __FUNCTION__ << "-> parse error line: " << line_number << " -> temperature_c out of range [-40, 120]" << std::endl;
        return false;
    }

    // gps_fix дорівнює 0 або 1;
    if (current_frame.gps_fix != 0 && current_frame.gps_fix != 1) {
        std::cerr << __FUNCTION__ << "-> parse error line: " << line_number << " -> gps_fix: not 0 or 1" << std::endl;
        return false;
    }

    // satellites >= 0
    if (current_frame.satellites < 0) {
        std::cerr << __FUNCTION__ << "-> parse error line: " << line_number << " -> satellites: < 0" << std::endl;
        return false;
    }

    // seq зростає на 1;
    if (line_number > 0) {
        if (current_frame.seq != previous_frame.seq + 1) {
            std::cerr << __FUNCTION__ << "-> parse error line: " << line_number << " -> seq: expected " << previous_frame.seq + 1 << ", but got " << current_frame.seq << std::endl;
            return false;
        }

        // timestamp_ms зростає;
        if (current_frame.timestamp_ms <= previous_frame.timestamp_ms) {
            std::cerr << __FUNCTION__ << "-> parse error line: " << line_number << " -> timestamp_ms: expected > " << previous_frame.timestamp_ms << ", but got " << current_frame.timestamp_ms << std::endl;
            return false;
        }
    }

    return true;
}

double compute_frame_rate_hz(const Frame frames[], int frame_count) {
    const long elapsed_ms = frames[frame_count - 1].timestamp_ms - frames[0].timestamp_ms;

    if (!elapsed_ms) {
        std::cerr << __FUNCTION__ << "-> error: elapsed_ms is 0" << std::endl;
        std::exit(4);
    }

    return static_cast<double>((frame_count - 1) * 1000 / elapsed_ms);
}

int read_frames(const char* path, Frame frames[], int max_frames) {
    std::ifstream input{path};
    if (!input) {
        std::cerr << __FUNCTION__ << "-> error: failed to open input file: " << path << std::endl;
        return 0;
    }

    int frame_count = 0;
    int error_code = 0;
    std::string error_message = "";
    char line[MAX_LINE_LENGTH];

    while (input.getline(line, MAX_LINE_LENGTH)) {
        if (line[0] == '\0') {
            continue;
        }

        if (frame_count < max_frames) {
            frames[frame_count] = parse_frame(line, error_code, error_message);
            if (error_code != 0) {
                std::cerr << __FUNCTION__ << "-> on parsing line: " << frame_count << " - " << error_message << std::endl;
                return 0;
            }

            Frame previous_frame = frame_count > 0 ? frames[frame_count - 1] : Frame{};
            if (!is_valid_frame(frame_count, frames[frame_count], previous_frame))
            {
                return 0;
            }
            ++frame_count;
        }
    }

    return frame_count;
}

Summary summarize(const Frame frames[], int frame_count) {
    Summary summary{};
    summary.frames_total = frame_count;
    summary.frames_valid = frame_count;
    summary.voltage_min = frames[0].voltage_v;
    summary.voltage_max = frames[0].voltage_v;
    summary.low_voltage_frames = 0;

    double temperature_sum = 0.0;

    for (int i = 0; i < frame_count; ++i) {
        if (frames[i].voltage_v < summary.voltage_min) {
            summary.voltage_min = frames[i].voltage_v;
        }

        if (frames[i].voltage_v > summary.voltage_max) {
            summary.voltage_max = frames[i].voltage_v;
        }

        temperature_sum += frames[i].temperature_c;

        if (frames[i].voltage_v < 22.0) {
            ++summary.low_voltage_frames;
        }
    }

    const int temperature_tenths = static_cast<int>(temperature_sum * 10.0) / frame_count;
    summary.temperature_avg = static_cast<double>(temperature_tenths) / 10.0;
    summary.frame_rate_hz = compute_frame_rate_hz(frames, frame_count);
    return summary;
}

void print_summary(const Summary& summary) {
    std::cout << "frames_total " << summary.frames_total << '\n';
    std::cout << "frames_valid " << summary.frames_valid << '\n';
    std::cout << "voltage_min " << summary.voltage_min << '\n';
    std::cout << "voltage_max " << summary.voltage_max << '\n';
    std::cout << "temperature_avg " << summary.temperature_avg << '\n';
    std::cout << "low_voltage_frames " << summary.low_voltage_frames << '\n';
    std::cout << "frame_rate_hz " << summary.frame_rate_hz << '\n';
}
