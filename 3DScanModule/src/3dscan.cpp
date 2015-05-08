/*******************************************************************************
INTEL CORPORATION PROPRIETARY INFORMATION
This software is supplied under the terms of a license agreement or nondisclosure
agreement with Intel Corporation and may not be copied or disclosed except in
accordance with the terms of that agreement
Copyright(c) 2014-2015 Intel Corporation. All Rights Reserved.
*******************************************************************************/
/// @file 3dscan.cpp
/// 3D Capture (PXC3DScan) sample application

// This sample advances from TARGETING to SCANNING at frame NUM_TARGETING_FRAMES...
static const size_t NUM_TARGETING_FRAMES = 75;
// ...and generates a mesh at frame (NUM_TARGETING_FRAMES + NUM_SCANNING_FRAMES)
static const size_t NUM_SCANNING_FRAMES = 250;

// The frame number and rate are shown in the stdout stream every REPORT_INTERVAL frames
static const unsigned REPORT_INTERVAL = 50;

// The thickness of the progress bar indicators in pixels
static const unsigned PROGRESS_BAR_THICKNESS_PIXELS = 5;

#include "pxc3dscan.h"
#include "pxcvideomodule.h"
#include "pxcsensemanager.h"
#include "pxcmetadata.h"
#include "util_cmdline.h"
#include "util_render.h"
#define NOMINMAX
#include <windows.h>
#include <conio.h>
//
#include<pclHeader.h>

#include <string>

//int loadPolygonFilelFromObj(std::wstring in_obj_filename, pcl::PolygonMesh& mesh_pcl);
int loadPolygonFilelFromObj(std::wstring in_obj_filename, pcl::PolygonMesh& mesh_pcl, pcl::PointCloud<pcl::Normal>::Ptr normal_cloud);

// Convert a wide Unicode string to an UTF8 string
std::string utf8_encode(const std::wstring &wstr)
{
    if( wstr.empty() ) return std::string();
    int size_needed = WideCharToMultiByte(CP_UTF8, 0, &wstr[0], (int)wstr.size(), NULL, 0, NULL, NULL);
    std::string strTo( size_needed, 0 );
    WideCharToMultiByte                  (CP_UTF8, 0, &wstr[0], (int)wstr.size(), &strTo[0], size_needed, NULL, NULL);
    return strTo;
}

// Convert an UTF8 string to a wide Unicode String
std::wstring utf8_decode(const std::string &str)
{
    if( str.empty() ) return std::wstring();
    int size_needed = MultiByteToWideChar(CP_UTF8, 0, &str[0], (int)str.size(), NULL, 0);
    std::wstring wstrTo( size_needed, 0 );
    MultiByteToWideChar                  (CP_UTF8, 0, &str[0], (int)str.size(), &wstrTo[0], size_needed);
    return wstrTo;
}


void ReportFrame(
    const PXC3DScan::Mode& mode,
    UtilRender* window,
    unsigned int& framenumber)
{
    const wchar_t* MODE = (mode == PXC3DScan::SCANNING) ? L"SCANNING" : L"TARGETING";
    if (window)
    {
        const int fps = window->GetCurrentFPS();
        if (fps > 0)
        {
            wprintf_s(L"%s frame %d at %d fps\n", MODE, framenumber, fps);
            return;
        }
    }
    wprintf_s(L"%s frame %d\n", MODE, framenumber);
}

bool PreviewImageContainsObject(PXCImage* preview_image)
{
    // If the preview image contains any color data, 
    // this return code is set to true.
    bool bPreviewContainsImageData = false;

    // Iterate over the pixels in the image
    PXCImage::ImageData preview_image_data;
    preview_image->AcquireAccess(PXCImage::ACCESS_READ,
        PXCImage::PIXEL_FORMAT_RGB24, &preview_image_data);
    const pxcI32 height = preview_image->QueryInfo().height;
    const pxcI32 width = preview_image->QueryInfo().width;
    const int PIXELS_PER_SAMPLE = 10;
    for (int y = 0; !bPreviewContainsImageData && y < height; y += PIXELS_PER_SAMPLE)
    {
        // Get the address of the row of pixels
        pxcBYTE* p = preview_image_data.planes[0]
            + y * preview_image_data.pitches[0];

        // For each pixel in the row...
        for (int x = 0; x < width; x++)
        {
            // ...determine if there is a valid image data
            if (p[0] | p[1] | p[2]) // We ignore the green channel due to green viewport boundaries which are shown in the preview.
            {
                // Ignore rendered frustum edge and the rendered progress bar
                if (!(p[0] == 51 && p[1] == 255 && p[2] == 102) && !(p[0] == 0 && p[1] == 255 && p[2] == 0) && !(p[0] == 0 && p[1] == 0 && p[2] == 255))
                {
                    bPreviewContainsImageData = true;
                    break;
                }
            }
            p += 3*PIXELS_PER_SAMPLE; // Move the pointer some number of pixel accross (BGR)
        }
    }
    preview_image->ReleaseAccess(&preview_image_data);

    return bPreviewContainsImageData;
}

void RenderProgressBar(
    PXCImage*             preview_image, 
    const PXC3DScan::Mode mode, 
    const bool            auto_switch,
    const bool            auto_exit,
    const size_t          frames_to_auto_switch)
{
    // Render a green (TARGETING) and red (SCANNING) progress bar
    PXCImage::ImageData preview_image_data;
    preview_image->AcquireAccess(PXCImage::ACCESS_READ_WRITE,
        PXCImage::PIXEL_FORMAT_RGB24, &preview_image_data);
    const size_t width = (size_t)preview_image->QueryInfo().width;

    // First, render the extent of the progress bar regions
    const size_t TARGETING_WIDTH = width * NUM_TARGETING_FRAMES / (NUM_TARGETING_FRAMES + NUM_SCANNING_FRAMES);
    for (size_t y = 0; y < PROGRESS_BAR_THICKNESS_PIXELS; y++)
    {
        // Get the address of the row of pixels to use
        pxcBYTE* p = preview_image_data.planes[0] + y * preview_image_data.pitches[0];
        size_t x;
        for (x = 0; x < TARGETING_WIDTH; x++, p+=3) {p[0]=0; p[1]=0xff; p[2]=0;}
        for (; x < width; x++, p+=3) {p[0]=0; p[1]=0; p[2]=0xff;}
    }

    // Then, the progress portion
    if (!auto_switch && !auto_exit) // When manual override is enabled, just indicate mode
    {
        const size_t offset = PROGRESS_BAR_THICKNESS_PIXELS * 2;
        for (size_t y = offset; y < offset + PROGRESS_BAR_THICKNESS_PIXELS; y++)
        {
            // Get the address of the row of pixels to use
            if (PXC3DScan::Mode::TARGETING == mode)
            {
                pxcBYTE* p = preview_image_data.planes[0] + y * preview_image_data.pitches[0];
                for (size_t x = 0; x < TARGETING_WIDTH; x++, p += 3) {p[0]=0; p[1]=0xff; p[2]=0;}
            }
            if (PXC3DScan::Mode::SCANNING == mode)
            {
                pxcBYTE* p = preview_image_data.planes[0] + y * preview_image_data.pitches[0] + TARGETING_WIDTH * 3;
                for (size_t x = TARGETING_WIDTH; x < width; x++, p += 3)  {p[0]=0; p[1]=0; p[2]=0xff;}
            }
        }
    }
    else // Show TARGETING and/or SCANNING progress.
    {
        const int offset = PROGRESS_BAR_THICKNESS_PIXELS * 2;
        for (int y = offset; y < offset + PROGRESS_BAR_THICKNESS_PIXELS; y++)
        {
            if (!auto_switch)
            {
                pxcBYTE* p = preview_image_data.planes[0] + y * preview_image_data.pitches[0];
                for (size_t x = 0; x < TARGETING_WIDTH; x++, p += 3)  {p[0]=0; p[1]=0xff; p[2]=0;}
                const size_t SCANNING_FRAMES = NUM_SCANNING_FRAMES - frames_to_auto_switch;
                const size_t SCANNING_WIDTH = width * NUM_SCANNING_FRAMES / (NUM_TARGETING_FRAMES + NUM_SCANNING_FRAMES);
                const size_t SCANNING_PROGRESS = SCANNING_WIDTH * SCANNING_FRAMES / NUM_SCANNING_FRAMES;
                for (size_t x = 0; x < SCANNING_PROGRESS; x++, p += 3)  {p[0]=0; p[1]=0; p[2]=0xff;}
            }
            else
            {
                const size_t TARGETING_FRAMES = NUM_TARGETING_FRAMES - frames_to_auto_switch;
                const size_t TARGETING_PROGRESS = TARGETING_WIDTH * TARGETING_FRAMES / NUM_TARGETING_FRAMES;
                pxcBYTE* p = preview_image_data.planes[0] + y * preview_image_data.pitches[0];
                for (size_t x = 0; x < TARGETING_PROGRESS; x++, p += 3)  {p[0]=0; p[1]=0xff; p[2]=0;}
            }
        }
    }
    preview_image->ReleaseAccess(&preview_image_data);
}

int wmain(int argc, WCHAR* argv[])
{
	// Report the name of the executable and the provided arguments to stdout
    wprintf_s(L"Starting %s", argv[0]);
    for (int arg = 1; arg < argc; arg++) wprintf_s(L" %s", argv[arg]);
    wprintf_s(L"\n\n");

    // Initialize the core system
    PXCSenseManager* pSenseManager = PXCSenseManager::CreateInstance();
    if (!pSenseManager)
    {
        wprintf_s(L"Error: PXCSenseManager is unavailable\n");
        return PXC_STATUS_ITEM_UNAVAILABLE;
    }

    // Process provided command line arguments. For example:
    // 1. Set the number of frames (e.g. 3dscan.exe -nframes 50)
    // 2. Stream redirection "from" file (e.g. 3dscan.exe -file myfile.ext)
    // 3. Stream redirection "to" file (e.g. 3dscan.exe -file myfile.ext -record)
    // 4. Enable object/ground detection (e.g. 3dscan.exe -object)
    UtilCmdLine cmdl(pSenseManager->QuerySession());
    if (!cmdl.Parse(L"-nframes-file-record-realtime-object", argc, argv)) return -1;

    // If a file was specified, configure the Capture Manager to use it.
    pxcStatus result = PXC_STATUS_NO_ERROR;
    if (cmdl.m_recordedFile)
    {
        result = pSenseManager->QueryCaptureManager()->
            SetFileName(cmdl.m_recordedFile, cmdl.m_bRecord);
        if (result < PXC_STATUS_NO_ERROR)
        {
            wprintf_s(L"Error: The specified file (%s) was not found (%d)\n",
                cmdl.m_recordedFile, result);
            return result;
        }
    }

    // Optional steps to send feedback to Intel Corporation to understand how often each SDK sample is used.
    PXCMetadata * md = pSenseManager->QuerySession()->QueryInstance<PXCMetadata>();
    if(md)
    {
        pxcCHAR sample_name[] = L"3D Scan";
        md->AttachBuffer(PXCSessionService::FEEDBACK_SAMPLE_INFO, (pxcBYTE*)sample_name, sizeof(sample_name));
    }

    // Enable the 3D Capture video module
    result = pSenseManager->Enable3DScan();
    if (result < PXC_STATUS_NO_ERROR)
    {
        wprintf_s(L"Error: Enable3DScan failed (%d)\n", result);
        return result;
    }
    PXC3DScan* pScanner = pSenseManager->Query3DScan();
    if (!pScanner) return PXC_STATUS_ITEM_UNAVAILABLE;

    // Initialize the streaming system
    result = pSenseManager->Init();
    if (result < PXC_STATUS_NO_ERROR)
    {
        wprintf_s(L"Error: Init failed (%d)\n", result);
        return result;
    }

    // Report the resulting stream source (from file or live)
    if (cmdl.m_recordedFile && !cmdl.m_bRecord)
    { // file
        wprintf_s(L"Streaming from %s \n", cmdl.m_recordedFile);
    }
    else
    { // live
        PXCCapture::DeviceInfo device_info;
        pSenseManager->QueryCaptureManager()->QueryCapture()->QueryDeviceInfo(0, &device_info);
        wprintf_s(L"Camera: %s \nFirmware: %d.%d.%d.%d \n",
            device_info.name,
            device_info.firmware[0], device_info.firmware[1],
            device_info.firmware[2], device_info.firmware[3]);
    }

    // Report the resulting profile
    {
        PXCVideoModule::DataDesc VideoProfile;
        result = pScanner->QueryInstance<PXCVideoModule>()->
            QueryCaptureProfile(PXCBase::WORKING_PROFILE, &VideoProfile);
        if (result < PXC_STATUS_NO_ERROR) return result;
        else
        {
            wprintf_s(L"Color: %dx%dx%0.f \nDepth: %dx%dx%0.f \n\n",
                VideoProfile.streams.color.sizeMax.width,
                VideoProfile.streams.color.sizeMax.height,
                VideoProfile.streams.color.frameRate.max,
                VideoProfile.streams.depth.sizeMax.width,
                VideoProfile.streams.depth.sizeMax.height,
                VideoProfile.streams.depth.frameRate.max);
        }
    }

    // Disable realtime mode if it was disabled using a cmd line arg (i.e. -realtime off)
    if (!cmdl.m_realtime)
    {
        pSenseManager->QueryCaptureManager()->SetRealtime(false);
        wprintf_s(L"<Realtime playback disabled> \n");
    }

    // Enable object on planar surface detection if "-object" provided as command line argument
    if (cmdl.m_bObject)
    {
        result = pScanner->SetTargetingOptions
            (PXC3DScan::TargetingOption::OBJECT_ON_PLANAR_SURFACE_DETECTION);
        if (result < PXC_STATUS_NO_ERROR)
        {
            wprintf_s(L"Error: SetTargetingOptions failed (%d)\n", result);
            return result;
        }
        wprintf_s(L"<Object on planar surface detection enabled> \n");
    }

    // Open the Preview windows and enter the main loop
    static PXC3DScan::ReconstructionOption reconstructionOptions = PXC3DScan::SOLIDIFICATION;
    {
        UtilRender window(L"DF_3DScan.exe: PXC3DScan C++ Sample");
        printf("<Preview window opened>\n");
        unsigned int framenumber = 1;
        ReportFrame(pScanner->QueryMode(), &window, framenumber);
        bool auto_switch = true;
        bool auto_exit = false;
        size_t frames_to_auto_switch = NUM_TARGETING_FRAMES;
        while (1)
        {
            // Wait for the next frame
            if (pSenseManager->AcquireFrame(true) < PXC_STATUS_NO_ERROR)
            {
                ReportFrame(pScanner->QueryMode(), NULL, framenumber);
                break; // Early exit if end of file reached
            }

            // Get the preview image for this frame
            PXCImage* preview_image = pScanner->AcquirePreviewImage();
            pSenseManager->ReleaseFrame();

            // Regularly report frame number and performance to stdout
            if (framenumber % REPORT_INTERVAL == 0) ReportFrame(pScanner->QueryMode(), &window, framenumber);

            // For user facing cameras, mirror the preview image to make it 
            // easier for the user to center the object in view.
            PXCCapture::DeviceInfo info;
            pSenseManager->QueryCaptureManager()->QueryDevice()->QueryDeviceInfo(&info);
            if (info.orientation == PXCCapture::DeviceOrientation::DEVICE_ORIENTATION_FRONT_FACING)
            {
                PXCImage::ImageData preview_image_data;
                preview_image->AcquireAccess(PXCImage::ACCESS_READ_WRITE,
                    PXCImage::PIXEL_FORMAT_RGB24, &preview_image_data);
                const pxcI32 height = preview_image->QueryInfo().height;
                const pxcI32 width = preview_image->QueryInfo().width;
                for (int y = 0; y < height; y++)
                {
                    // Get the address of the row of pixels
                    pxcBYTE* p = preview_image_data.planes[0]
                        + y * preview_image_data.pitches[0];
                    pxcBYTE* pr = p + ((width - 1) * 3);

                    // For each pixel in the row 
                    for (int x = 0; x < width/2; x++)
                    {
                        pxcBYTE temp;
                        for (unsigned c = 0; c < 3; c++)
                        {
                            temp = p[c];
                            p[c] = pr[c];
                            pr[c] = temp;
                        }
                        p += 3;
                        pr -= 3;
                    }
                }
                preview_image->ReleaseAccess(&preview_image_data);
            }

            // Add the progress bar to the preview image
            RenderProgressBar(preview_image, pScanner->QueryMode(), auto_switch, auto_exit, frames_to_auto_switch);

            // Render and then release the preview image
            if (!window.RenderFrame(preview_image))
            {
                if (framenumber % REPORT_INTERVAL != 0) ReportFrame(pScanner->QueryMode(), NULL, framenumber);
                printf("<Preview window closed>\n");
                break; // Early exit if end of file reached
            }
            preview_image->Release();

            if (GetAsyncKeyState(VK_ESCAPE) && !GetAsyncKeyState(VK_CONTROL)) // Early exit if Esc key pressed
            {
                if (framenumber % REPORT_INTERVAL != 0) ReportFrame(pScanner->QueryMode(), NULL, framenumber);
                printf("<Esc key pressed>\n");
                break;
            }

            // Toggle object/surface detection when F4 is released
            static bool bObjectSurfaceEnabled = false;
            static bool f4_key_pressed_previous_frame = false;
            bool f4_key_pressed = GetAsyncKeyState(VK_F4) ? true : false;
            if (f4_key_pressed_previous_frame && !f4_key_pressed)
            {
                bObjectSurfaceEnabled ^= true;
                if (!bObjectSurfaceEnabled)
                {
                    pScanner->SetTargetingOptions(PXC3DScan::NO_TARGETING_OPTIONS);
                    wprintf_s(L"<Object on planar surface detection disabled>\n");
                }
                else
                {
                    pScanner->SetTargetingOptions(PXC3DScan::OBJECT_ON_PLANAR_SURFACE_DETECTION);
                    wprintf_s(L"<Object on planar surface detection enabled>\n");
                }
            }
            f4_key_pressed_previous_frame = f4_key_pressed;

            // Toggle solidification when F5 is released
           // static bool f5_key_pressed_previous_frame = false;
          //  bool f5_key_pressed = GetAsyncKeyState(VK_F5) ? true : false;
           // if (f5_key_pressed_previous_frame && !f5_key_pressed)
            //{
                //reconstructionOptions = reconstructionOptions ^ PXC3DScan::SOLIDIFICATION;
                if (reconstructionOptions & PXC3DScan::SOLIDIFICATION)
                {
                    wprintf_s(L"<Solidification enalbed>\n");
                }
                else wprintf_s(L"<Solidification disabled>\n");
            //}
            //f5_key_pressed_previous_frame = f5_key_pressed;

            if (auto_switch)
            {
                bool bObjectInScanningVolume = PreviewImageContainsObject(pScanner->AcquirePreviewImage());
                if (!bObjectInScanningVolume)
                {
                    if (framenumber % REPORT_INTERVAL == 0) printf("<Move forward, into the scanning area to start>\n");
                }
                else if (!--frames_to_auto_switch)
                {
                    if (pScanner->SetMode(PXC3DScan::SCANNING) != PXC_STATUS_ITEM_UNAVAILABLE)
                    {
                        printf("<Auto-switch elapsed>\n");
                        if (framenumber % REPORT_INTERVAL != 0) ReportFrame(pScanner->QueryMode(), NULL, framenumber);
                        // Switch to auto-exit mode
                        frames_to_auto_switch = NUM_SCANNING_FRAMES;
                        auto_switch = false;
                        auto_exit = true;
                    }
                    else frames_to_auto_switch++; // retry
                }
            }
            if (auto_exit && !--frames_to_auto_switch)
            {
                if (framenumber % REPORT_INTERVAL != 0) ReportFrame(pScanner->QueryMode(), NULL, framenumber);
                printf("<Auto-exit elapsed>\n");
                break;
            }

            // Space bar is used to manually switch between modes and disable Auto-switch/exit
            if (window.m_pause)
            {
                printf("<Spacebar pressed>\n");
                window.m_pause = false; // Reset the trigger

                // Toggle the mode
                if (pScanner->QueryMode() == PXC3DScan::TARGETING)
                {
                    result = pScanner->SetMode(PXC3DScan::SCANNING);
                    if (result == PXC_STATUS_ITEM_UNAVAILABLE)
                    {
                        wprintf_s(L"<Object on planar surface not detected>\n", result);
                    }
                }
                else pScanner->SetMode(PXC3DScan::TARGETING);

                // Report the frame of the mode transition
                if (framenumber % REPORT_INTERVAL != 0) ReportFrame(pScanner->QueryMode(), &window, framenumber);

                // Disable auto transitions
                auto_switch = false;
                auto_exit = false;
            }

            framenumber++; // Advance to the next frame
        }
    }
	
    // Setup the output path to a writable location.
    size_t unused;
    WCHAR* pUserProfilePath;
    _wdupenv_s(&pUserProfilePath, &unused, L"USERPROFILE");
    const PXC3DScan::FileFormat format = PXC3DScan::OBJ; // OBJ, PLY or STL
    const pxcCHAR* ext = PXC3DScan::FileFormatToString(format);
    const size_t FSIZE = 4096;
    WCHAR filename[FSIZE];
    swprintf_s(filename, FSIZE, L"%s\\Documents\\3dscan.%s", pUserProfilePath, ext);
    // If applicable, reconstruct the 3D Mesh to the specific file/format
    result = PXC_STATUS_NO_ERROR;
    bool bMeshSaved = false;
    if (pScanner->QueryMode() == PXC3DScan::SCANNING)
    {
        wprintf_s(L"Generating %s...", filename);
        result = pScanner->Reconstruct(format, filename, reconstructionOptions);
        if (result >= PXC_STATUS_NO_ERROR)
        {
            bMeshSaved = true;
            wprintf_s(L"done.\n");
        }
        else if (result == PXC_STATUS_FEATURE_UNSUPPORTED) wprintf_s(L"SOLIDIFICATION requires write access to current working directory in this release. See release notes for details.\n", filename);
        else if (result == PXC_STATUS_FILE_WRITE_FAILED) wprintf_s(L"the file could not be created using the provided file path.\n", filename);
        else if (result == PXC_STATUS_ITEM_UNAVAILABLE) wprintf_s(L"save aborted due to empty scanning volume.\n", filename);	
    }
    else wprintf_s(L"Empty scanning volume.\n");

    pSenseManager->Release();
	
    if (bMeshSaved)
    {
        printf("File Generated...\n");
		
        pcl::PolygonMesh mesh; // point cloud with colSor
		pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>);; // point normals
		//const std::string fileName = "scanME.obj";//utf8_encode(filename);
        //pcl::io::loadPolygonFileSTL(fileName,mesh);
		std::wstring fileName(filename);
		int E1 = loadPolygonFilelFromObj(fileName,mesh,normal_cloud);
        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>); 
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>); 
        pcl::fromPCLPointCloud2(mesh.cloud, *cloud); 
        int E2 = pcl::io::savePCDFileASCII("D:\\SaveME2.pcd",*cloud); 
		wprintf_s(L"Error = %d , %d",E1, E2);
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
		viewer = normalsVis(mesh,cloud,normal_cloud);
		printf("Error = %d , %d",E1, E2);
		//--------------------
		// -----Main loop-----
		//--------------------
		while (!viewer->wasStopped ())
		{
			viewer->spinOnce (100);
			boost::this_thread::sleep (boost::posix_time::microseconds (100000));
		}
    }

    return 0; //(int)result
}
