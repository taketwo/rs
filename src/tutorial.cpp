/*******************************************************************************

INTEL CORPORATION PROPRIETARY INFORMATION
This software is supplied under the terms of a license agreement or nondisclosure
agreement with Intel Corporation and may not be copied or disclosed except in
accordance with the terms of that agreement
Copyright(c) 2013-2014 Intel Corporation. All Rights Reserved.

*******************************************************************************/
/* 
Description:
This is the raw streams procedural sample that shows how to capture color and depth synchronized by procedural calls. 

*/

#include <windows.h>
#include <wchar.h>
#include <pxcsensemanager.h>
//#include "util_render.h"  //SDK provided utility class used for rendering (packaged in libpxcutils.lib)

// maximum number of frames to process if user did not close the rendering window
#define MAX_FRAMES 5000 

int wmain(int argc, WCHAR* argv[]) {

	// initialize the util render 
	//UtilRender *renderColor  = new UtilRender(L"COLOR STREAM");
	//UtilRender *renderDepth  = new UtilRender(L"DEPTH STREAM");

	// create the PXCSenseManager
	PXCSenseManager *psm=0;
	psm = PXCSenseManager::CreateInstance();
	if (!psm) {
		wprintf_s(L"Unable to create the PXCSenseManager\n");
		return 1;
	}

	// select the color stream of size 640x480 and depth stream of size 320x240
	psm->EnableStream(PXCCapture::STREAM_TYPE_COLOR, 640, 480);
	psm->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, 640, 480);

	// initialize the PXCSenseManager
	if(psm->Init() != PXC_STATUS_NO_ERROR) return 2;

	PXCImage *colorIm, *depthIm;
	for (int i=0; i<MAX_FRAMES; i++) {

		// This function blocks until all streams are ready (depth and color)
		// if false streams will be unaligned
		if (psm->AcquireFrame(true)<PXC_STATUS_NO_ERROR) break; 

		// retrieve all available image samples
		PXCCapture::Sample *sample = psm->QuerySample();

		// retrieve the image or frame by type from the sample
		colorIm = sample->color;
		depthIm = sample->depth;

		// render the frame
		//if (!renderColor->RenderFrame(colorIm)) break;
		//if (!renderDepth->RenderFrame(depthIm)) break;

		// release or unlock the current frame to fetch the next frame
		psm->ReleaseFrame();
	}

	// delete the UtilRender instance
	//delete renderColor;
	//delete renderDepth;

	// close the last opened streams and release any session and processing module instances
	psm->Release();
	
	return 0;
}