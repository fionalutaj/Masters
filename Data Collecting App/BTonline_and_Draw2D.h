//DrawSkeleton
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>

#include <k4a/k4a.h>
#include <k4a/k4a.hpp>

#include <k4abt.h>
#include <opencv2/opencv.hpp>
#include <math.h>

//BTonline
#include <string>
#include <iomanip>
#include <filesystem>

#include <stdio.h>
#include <stdlib.h>


#include <nlohmann/json.hpp>
#include "C:\Motion Tracking\ConsoleApplication2\Azure-Kinect-Samples-master\body-tracking-samples\sample_helper_includes\BodyTrackingHelpers.h"

#include "C:\Motion Tracking\ConsoleApplication2\Azure-Kinect-Samples-master\body-tracking-samples\sample_helper_includes\Utilities.h"


using namespace std;
using namespace nlohmann;

//using std::filesystem::current_path;

#define VERIFY(result, error)                                                                            \
    if(result != K4A_RESULT_SUCCEEDED)                                                                   \
    {                                                                                                    \
        printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__); \
        exit(1);                                                                                         \
    }
\

///// INTEGRATED FUNCTION /////
bool skeletal_tracking(json& frames_json, int frame_count, k4abt_frame_t body_frame, k4a_calibration_t sensor_calibration)
{
    uint32_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
    uint64_t timestamp = k4abt_frame_get_device_timestamp_usec(body_frame);

    // Get UNIX time
    const auto p1 = std::chrono::system_clock::now();
    uint64_t timestamp_unix = std::chrono::duration_cast<std::chrono::microseconds>(p1.time_since_epoch()).count();

    json frame_result_json;
    frame_result_json["timestamp_unix"] = timestamp_unix;
    frame_result_json["frame_id"] = frame_count;
    frame_result_json["num_bodies"] = num_bodies;
    frame_result_json["bodies"] = json::array();

    for (uint32_t i = 0; i < num_bodies; i++)
    {

        k4abt_skeleton_t skeleton;
        VERIFY(k4abt_frame_get_body_skeleton(body_frame, i, &skeleton), "Get body from body frame failed!");

        json body_result_json;

        int body_id = k4abt_frame_get_body_id(body_frame, i);
        body_result_json["body_id"] = body_id;

        body_result_json["joint_positions"] = json::array();
        for (int j = 0; j < (int)K4ABT_JOINT_COUNT; j++)
        {
            body_result_json["joint_positions"].push_back({
                skeleton.joints[j].position.xyz.x,
                skeleton.joints[j].position.xyz.y,
                skeleton.joints[j].position.xyz.z
                });

            body_result_json["joint_confidence"].push_back(skeleton.joints[j].confidence_level);
        }
        frame_result_json["bodies"].push_back(body_result_json);
    }

    frames_json.push_back(frame_result_json);

    return true;
}
