#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <filesystem>
#include <windows.h>

#include <stdio.h>
#include <stdlib.h>

#include <k4a/k4a.h>
#include <k4abt.h>

#include <nlohmann/json.hpp>
#include "C:\Motion Tracking\ConsoleApplication2\Azure-Kinect-Samples-master\body-tracking-samples\sample_helper_includes\BodyTrackingHelpers.h"

#include "C:\Motion Tracking\ConsoleApplication2\Azure-Kinect-Samples-master\body-tracking-samples\sample_helper_includes\Utilities.h"

#include <opencv2/opencv.hpp>
#include "BTonline_and_Draw2D.h"
#include <chrono>  
#include <ctime>  

using json = nlohmann::json;
#define VERIFY(result, error)                                                                            \
    if(result != K4A_RESULT_SUCCEEDED)                                                                   \
    {                                                                                                    \
        printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__); \
        exit(1);                                                                                         \
    }                                                                                                    \
extern bool skeletal_tracking(json& frames_json, int frame_count, k4abt_frame_t body_frame, k4a_calibration_t sensor_calibration);



void RestartApplication() {
    STARTUPINFO si = { sizeof(STARTUPINFO) };
    PROCESS_INFORMATION pi;
    TCHAR szPath[MAX_PATH];
    GetModuleFileName(NULL, szPath, MAX_PATH); // Gets the current executable's path

    // Start a new instance of the current application
    if (!CreateProcess(szPath,   // No module name (use command line)
        NULL,        // Command line
        NULL,           // Process handle not inheritable
        NULL,           // Thread handle not inheritable
        FALSE,          // Set handle inheritance to FALSE
        0,              // No creation flags
        NULL,           // Use parent's environment block
        NULL,           // Use parent's starting directory 
        &si,            // Pointer to STARTUPINFO structure
        &pi)           // Pointer to PROCESS_INFORMATION structure
        ) {
        printf("Failed to restart application. Error: %d\n", GetLastError());
    }
    exit(0); // Exit the current instance
}

// Function to save JSON data to a file  
int main()
{
    auto start_time = std::chrono::steady_clock::now();


    HWND consolewindow = GetConsoleWindow();
    ShowWindow(consolewindow, SW_MAXIMIZE);
    //SetWindowPos(consolewindow, 0, 300, 531, 0, 0, SWP_NOSIZE | SWP_NOZORDER);
    //RECT r;
    //GetWindowRect(consolewindow, &r);
    //MoveWindow(consolewindow, r.left, r.top, 800, 350, TRUE);

    system("color 2f");

    //num devices
    uint32_t DeviceCount = k4a_device_get_installed_count();
    //std::cout << "number of k4a devices: " << DeviceCount << "\n" << std::endl;


    for (uint32_t i = 0; i < DeviceCount; i++)

    {
        k4a_device_t device_trial = NULL;

        if (K4A_FAILED(k4a_device_open(i, &device_trial)))
        {
            std::cout << "k4a device with the index " << i << " is unavailable\n" << std::endl;
            RestartApplication();

            continue;
        }

        else
        {
            //get serial num
            size_t serial_size_trial = 0;
            k4a_device_get_serialnum(device_trial, NULL, &serial_size_trial);

            char* serial_trial = (char*)(malloc(serial_size_trial));
            k4a_device_get_serialnum(device_trial, serial_trial, &serial_size_trial);
            std::cout << "serial number: " << serial_trial << "; to select, press " << i << std::endl;
            free(serial_trial);

            k4a_device_close(device_trial);
        }

    }

    //insert index
    k4a_device_t device = NULL;
    if (K4A_FAILED(k4a_device_open(0, &device)))
    {
        std::cout << "failed to open k4a device\n" << std::endl;
        RestartApplication(); // Restart instead of exiting

    }

    size_t serial_size = 0;
    k4a_device_get_serialnum(device, NULL, &serial_size);
    char* serial = (char*)(malloc(serial_size));
    k4a_device_get_serialnum(device, serial, &serial_size);
    std::cout << "k4a device " << serial << " successfully initiated." << std::endl;

    //define output file
    // Define output file with timestamp
    //initiate JSON, recored serial num
    json json_output;
    json_output["k4a_device_serial_number"] = serial;
    free(serial);

    //config, make sure depth camera is enabled.
    k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    deviceConfig.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_OFF;//ensures that depth and color images are both available in the capture

    //open cam
    if (K4A_FAILED(k4a_device_start_cameras(device, &deviceConfig)))
    {
        std::cout << "failed to start cameras." << std::endl;
        k4a_device_close(device);
        return -1;
    }
    std::cout << "\n" << "device successifully activated." << "\n" << std::endl;

    //get calibration
    k4a_calibration_t sensor_calibration;
    k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensor_calibration);
    VERIFY(k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensor_calibration), "get depth calibration failed.");

    //creat tracker
    k4abt_tracker_t tracker = NULL;
    k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;

    tracker_config.processing_mode = k4abt_tracker_processing_mode_t::K4ABT_TRACKER_PROCESSING_MODE_CPU;
    //tracker_config.model_path = "C:\\Program Files\\Azure Kinect Body Tracking SDK\\tools\\directml.dll";
    VERIFY(k4abt_tracker_create(&sensor_calibration, tracker_config, &tracker), "body tracker initilisation failed.");


    //store all joint names to the json
    json_output["joint_names"] = json::array();
    for (int i = 0; i < (int)K4ABT_JOINT_COUNT; i++)
    {
        json_output["joint_names"].push_back(g_jointNames.find((k4abt_joint_id_t)i)->second);
    }

    //store all bone linkings to the json
    json_output["bone_list"] = json::array();
    for (int i = 0; i < (int)g_boneList.size(); i++)
    {
        json_output["bone_list"].push_back({ g_jointNames.find(g_boneList[i].first)->second,
                                             g_jointNames.find(g_boneList[i].second)->second });
    }


    int frame_count = 0;
    bool success = true;
    bool recording = true;
    json frames_json;
    std::string basePath = "C:/Motion Tracking/ConsoleApplication2/Data Collection/";
    int last_saved_frame_count = 0;


    while (true)
    {
        k4a_capture_t sensor_capture;
        k4a_wait_result_t get_capture_result = k4a_device_get_capture(device, &sensor_capture, K4A_WAIT_INFINITE);

        if (get_capture_result == K4A_WAIT_RESULT_SUCCEEDED)
        {
            if (recording)
            {
                frame_count++;
                
                // Enqueue the capture for body tracking
                k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker, sensor_capture, K4A_WAIT_INFINITE);
                if (queue_capture_result == K4A_WAIT_RESULT_FAILED)
                {
                    std::cerr << "Error! Adding capture to tracker process queue failed!" << std::endl;
                    k4a_capture_release(sensor_capture);
                    break;
                }

                // Pop the result from body tracker
                k4abt_frame_t body_frame = nullptr;
                k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_INFINITE);
                uint32_t num_bodies = k4abt_frame_get_num_bodies(body_frame);

                if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
                {
                    // Call your skeletal_tracking function
                    if (num_bodies > 0) {
                        last_saved_frame_count = 0;
                        success = skeletal_tracking(frames_json, frame_count, body_frame, sensor_calibration);

                    }

                    if (num_bodies == 0) 
                    {
                        std::cout << "No bodies detected " << std::endl;
                        // No bodies detected, continue without breaking
                        //std::cout << "No bodies detected. Continuing..." << std::endl;

                        // Modify here to simply continue to the next iteration
                        //k4abt_frame_release(body_frame); // Release the body frame once done
                        //k4a_capture_release(sensor_capture); // Release the sensor capture

                        
                        if (last_saved_frame_count == 0 && (!frames_json.empty())) {
                            auto now = std::chrono::system_clock::now();
                            std::time_t timestamp = std::chrono::system_clock::to_time_t(now);

                            char fileName[256];
                            strftime(fileName, sizeof(fileName), "%Y%m%d_%H%M%S", std::localtime(&timestamp));



                            json_output["frames"] = frames_json;
                            //std::cout << "DONE " << std::endl;

                            const char* path = "C:/Motion Tracking/ConsoleApplication2/Data Collection/";


                            char   output_path[256];
                            strncpy_s(output_path, path, sizeof(output_path));
                            strncat_s(output_path, fileName, sizeof(output_path));

                            std::ofstream output_file(output_path);
                            output_file << std::setw(4) << json_output << std::endl;
                            std::cout << "Results saved in " << path << std::endl;
                            last_saved_frame_count = 1;
                            frames_json.clear();
                        }
                        
                        continue;
                        // Skip the rest of the loop and continue to the next frame

                    }

                    k4abt_frame_release(body_frame); // Release the body frame once done
                }
                else
                {
                    std::cerr << "Error! Popping body tracking result failed!" << std::endl;
                }

                k4a_capture_release(sensor_capture); // Release the sensor capture
                auto current_time = std::chrono::steady_clock::now();
                std::chrono::duration<double> elapsed = current_time - start_time;


                if (elapsed.count() >= 120)
                {
                    auto now = std::chrono::system_clock::now();
                    std::time_t timestamp = std::chrono::system_clock::to_time_t(now);

                    char fileName[256];
                    strftime(fileName, sizeof(fileName), "%Y%m%d_%H%M%S", std::localtime(&timestamp));

                    //initiate JSON, recored serial num


                    json_output["frames"] = frames_json;
                    //std::cout << "DONE " << std::endl;

                    const char* path = "C:/Motion Tracking/ConsoleApplication2/Data Collection/";


                    char   output_path[256];
                    strncpy_s(output_path, path, sizeof(output_path));
                    strncat_s(output_path, fileName, sizeof(output_path));

                    std::ofstream output_file(output_path);
                    output_file << std::setw(4) << json_output << std::endl;
                    std::cout << "Results saved in " << path << std::endl;
                    start_time = current_time;
                    frames_json.clear();


                }
            }
        }
        /*
        else if (get_capture_result == K4A_WAIT_RESULT_TIMEOUT)
        {
            printf("Error! Get depth frame time out!\n");
            break;
        }
        else
        {
            printf("Get depth capture returned error: %d\n", get_capture_result);
            break;
        }
        */
    }


    if (success && (!frames_json.empty()))
    {
        auto now = std::chrono::system_clock::now();
        std::time_t timestamp = std::chrono::system_clock::to_time_t(now);

        char fileName[256];
        strftime(fileName, sizeof(fileName), "%Y%m%d_%H%M%S", std::localtime(&timestamp));



        json_output["frames"] = frames_json;
        //std::cout << "DONE " << std::endl;

        const char* path = "C:/Motion Tracking/ConsoleApplication2/Data Collection/";

        char output_path[256];
        strncpy_s(output_path, path, sizeof(output_path));
        strncat_s(output_path, fileName, sizeof(output_path));

        std::ofstream output_file(output_path);
        output_file << std::setw(4) << json_output << std::endl;
        std::cout << "Results saved in " << path << std::endl;

    }


    //printf("finished body tracking processing\n");

    //k4abt_tracker_shutdown(tracker);
    //k4abt_tracker_destroy(tracker);
   // k4a_device_stop_cameras(device);
    //k4a_device_close(device);

    return 0;
}
