/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (C) 2016, Magazino GmbH. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Magazino GmbH nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

/*
This program will open a Lucid Arena Camera and write a desired camera id.
*/

// STD
#include <unistd.h>
#include <algorithm>
#include <string>

// Arena
#include <ArenaApi.h>

int main(int argc, char* argv[])
{
  if (argc < 2)
  {
    std::cerr << "ERROR: No device_user_id set!" << std::endl;
    std::cout << "USAGE: write_device_user_id_to_camera DEVICE_USER_ID" << std::endl;
    return 1;
  }

  std::string desired_device_user_id(reinterpret_cast<char*>(argv[1]));

  if (desired_device_user_id.empty())
  {
    std::cout << "ERROR:" << std::endl;
    std::cout << "Your desired device_user_id is empty!" << std::endl;
    return 2;
  }
  try
  {
    // Create an instant camera object with the camera device found first.
    Arena::ISystem* pSystem = Arena::OpenSystem();
    pSystem->UpdateDevices(100);

    std::vector<Arena::DeviceInfo> deviceInfos = pSystem->GetDevices();
    if (deviceInfos.size() == 0)

    {
      std::cout << "ERROR:" << std::endl;
      std::cout << "No camera connected, unable to write to camera" << std::endl;
      Arena::CloseSystem(pSystem);
      return 2;
    }

    Arena::IDevice* pDevice = pSystem->CreateDevice(deviceInfos[0]);

    GenApi::INodeMap* pNodeMap = pDevice->GetNodeMap();
    GenApi::CStringPtr current_device_user_id = pNodeMap->GetNode("DeviceUserID");
    current_device_user_id->SetValue(GenICam::gcstring(desired_device_user_id.c_str()));

    std::cout << "Successfully wrote new device user id: " << current_device_user_id->GetValue() << " to the camera "
              << deviceInfos[0].ModelName() << std::endl;

    pSystem->DestroyDevice(pDevice);
    Arena::CloseSystem(pSystem);
  }

  catch (GenICam::GenericException& e)
  {
    // Error handling.
    std::cerr << "An exception occurred." << std::endl << e.GetDescription() << std::endl;
    return 3;
  }

  return 0;
}
