/*
 * Copyright (C) 2018  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>

namespace color_limits {
    /*cv::Scalar const YELLOW_UPPER_HSV(20, 190, 220);
    cv::Scalar const YELLOW_LOWER_HSV(14, 100, 120);
    cv::Scalar const BLUE_UPPER_HSV(145, 255, 200);
    cv::Scalar const BLUE_LOWER_HSV(100, 120, 30);
    cv::Scalar const ORANGE_UPPER_HSV(10, 265, 295);
    cv::Scalar const ORANGE_LOWER_HSV(-10, 245, 215);
    cv::Scalar const CAR_BACK_UPPER_HSV(179,255, 148);
    cv::Scalar const CAR_BACK_LOWER_HSV(0, 228, 20);
    */
    
//simulation
    cv::Scalar const BLUE_UPPER_HSV(124, 255, 255);
    cv::Scalar const BLUE_LOWER_HSV(110, 157, 255);
    cv::Scalar const YELLOW_UPPER_HSV(35, 255, 255);
    cv::Scalar const YELLOW_LOWER_HSV(9, 255, 255);
    cv::Scalar const ORANGE_UPPER_HSV(10, 265, 295);
    cv::Scalar const ORANGE_LOWER_HSV(-10, 245, 215);
    cv::Scalar const CAR_BACK_UPPER_HSV(187, 265, 245);
    cv::Scalar const CAR_BACK_LOWER_HSV(167, 245, 165);
    
};

std::vector<cv::Rect> findBoundingBox(std::vector<std::vector<cv::Point>> contours){
    std::vector<cv::Rect> boundingBoxes;
    for(auto &contour : contours){
        cv::Rect boundingBox = cv::boundingRect(contour);
        
        boundingBoxes.push_back(boundingBox);
    }
    return boundingBoxes;
}

cv::Mat findCones(cv::Mat &hsv, cv::Scalar lower_hsv, cv::Scalar upper_hsv) {
    cv::Mat cones;
    cv::medianBlur(hsv,hsv,11);
    cv::inRange(hsv, lower_hsv, upper_hsv, cones);

    uint32_t const iterations = 3;

    
    for (uint32_t i = 0; i < iterations; i++) {
        cv::dilate(cones, cones, cv::Mat(), cv::Point(-1, -1), iterations, 1, 1);
        cv::erode(cones, cones, cv::Mat(), cv::Point(-1, -1), iterations, 1, 1);   
    }
    return cones;
}

int32_t main(int32_t argc, char **argv) {
    int32_t retCode{1};
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ( (0 == commandlineArguments.count("cid")) ||
         (0 == commandlineArguments.count("name")) ||
         (0 == commandlineArguments.count("width")) ||
         (0 == commandlineArguments.count("height")) ) {
        std::cerr << argv[0] << " attaches to a shared memory area containing an ARGB image." << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --cid=<OD4 session> --name=<name of shared memory area> [--verbose]" << std::endl;
        std::cerr << "         --cid:    CID of the OD4Session to send and receive messages" << std::endl;
        std::cerr << "         --name:   name of the shared memory area to attach" << std::endl;
        std::cerr << "         --width:  width of the frame" << std::endl;
        std::cerr << "         --height: height of the frame" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=112 --name=img.argb --width=640 --height=480 --verbose" << std::endl;
    }
    else {
        const std::string NAME{commandlineArguments["name"]};
        const uint32_t WIDTH{static_cast<uint32_t>(std::stoi(commandlineArguments["width"]))};
        const uint32_t HEIGHT{static_cast<uint32_t>(std::stoi(commandlineArguments["height"]))};
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};

        // Attach to the shared memory.
        std::unique_ptr<cluon::SharedMemory> sharedMemory{new cluon::SharedMemory{NAME}};
        if (sharedMemory && sharedMemory->valid()) {
            std::clog << argv[0] << ": Attached to shared memory '" << sharedMemory->name() << " (" << sharedMemory->size() << " bytes)." << std::endl;

            // Interface to a running OpenDaVINCI session; here, you can send and receive messages.
            cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

            u_int64_t frameCounter{0};
            
            // Endless loop; end the program by pressing Ctrl-C.
            while (od4.isRunning()) {
                cv::Mat img;

                // Wait for a notification of a new frame.
                sharedMemory->wait();

                // Lock the shared memory.
                sharedMemory->lock();
                {
                    
                    cv::Mat wrapped(HEIGHT, WIDTH, CV_8UC4, sharedMemory->data());
                    img = wrapped.clone();
                }
                sharedMemory->unlock();

                
                
                frameCounter++;

                // filter out the hood of the car.
                cv::Point front[1][4];
                front[0][0] = cv::Point(0,img.rows);
                front[0][1] = cv::Point(460,610);
                front[0][2] = cv::Point(860,610);
                front[0][3] = cv::Point(img.cols,img.rows);
                cv::fillConvexPoly(img, front[0], 4, cv::Scalar(0,0,0));
                // remove top part of the image in order to remove false positives.
                int x = 0;
                int y = img.rows/2;
                
                img(cv::Rect(x,y,img.cols,img.rows-y)).copyTo(img);
                
                
                // Convert to HSV color space.
                cv::Mat hsv;
                cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

                cv::Mat blueCones = findCones(hsv, color_limits::BLUE_LOWER_HSV, color_limits::BLUE_UPPER_HSV);
                cv::Mat yellowCones = findCones(hsv, color_limits::YELLOW_LOWER_HSV, color_limits::YELLOW_UPPER_HSV);
                cv::Mat orangeCones = findCones(hsv, color_limits::ORANGE_LOWER_HSV, color_limits::ORANGE_UPPER_HSV);
                cv::Mat kiwiCarBack = findCones(hsv, color_limits::CAR_BACK_LOWER_HSV, color_limits::CAR_BACK_UPPER_HSV);


                std::vector<std::vector<cv::Point>> contoursBlue;
                std::vector<std::vector<cv::Point>> contoursYellow;
                std::vector<std::vector<cv::Point>> contoursOrange;
                std::vector<std::vector<cv::Point>> contoursCarBack;
                std::vector<cv::Vec4i> hierarchy;
                
                cv::findContours(blueCones, contoursBlue, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
                cv::findContours(yellowCones, contoursYellow, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
                cv::findContours(orangeCones, contoursOrange, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
                cv::findContours(kiwiCarBack, contoursCarBack, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

                std::vector<cv::Rect> blueBox = findBoundingBox(contoursBlue);
                std::vector<double> distanceBlue;
                std::vector<cv::Point> bluePointsInRange;
                for(auto &box : blueBox){
                    cv::Scalar const blue(255, 255, 0);
                    cv::rectangle(img, box, blue);
                    cv::Point center(box.x + box.width/2,box.y+box.height);
                    circle( img,center,5,cv::Scalar( 255, 255, 0),cv::FILLED,cv::LINE_8 );
                      
                    cv::Point newPointBlue(center.x-(img.cols/2), img.rows-center.y);
                    bluePointsInRange.push_back(newPointBlue);
                    
                }
                int blueMinY = 10000;
                int blueXValue = 10000;
                for (auto &point : bluePointsInRange)
                    {
                        if (point.y < blueMinY){
                            blueMinY = point.y;
                            blueXValue = point.x;
                        }
                    }
                cv::Point closestPointBlue(blueXValue,blueMinY);
                std::cout <<"Closest point blue: " << closestPointBlue <<std::endl;
                cluon::data::TimeStamp sampleTime = cluon::time::now();
                opendlv::logic::perception::Cones blueConesMsg;
                blueConesMsg.x(closestPointBlue.x);
                blueConesMsg.y(closestPointBlue.y);
                od4.send(blueConesMsg, sampleTime,0);


                std::vector<cv::Rect> yellowBox = findBoundingBox(contoursYellow);
                std::vector<double> distanceYellow;
                std::vector<cv::Point> yellowPointsInRange;
                for(auto &box : yellowBox){
                    cv::Scalar const yellow(0, 255, 255);
                    cv::rectangle(img, box, yellow);
                    cv::Point center(box.x + box.width/2,box.y+box.height);
                      circle( img,center,5,cv::Scalar( 0, 255, 255),cv::FILLED,cv::LINE_8 );
                      cv::Point newPointYellow(center.x-(img.cols/2), img.rows-center.y);
                      yellowPointsInRange.push_back(newPointYellow);
                      
                }
                int yellowMinY = 10000;
                int yellowXValue = 10000;
                for (auto &point : yellowPointsInRange)
                    {
                        if (point.y < yellowMinY){
                            yellowMinY = point.y;
                            yellowXValue = point.x;
                        
                        }
                    }
                cv::Point closestPointYellow(yellowXValue,yellowMinY);
                std::cout <<"Closest point yellow: " << closestPointYellow <<std::endl;
                std::cout <<" "<<std::endl;
                opendlv::logic::perception::Cones yellowConesMsg;
                yellowConesMsg.x(closestPointYellow.x);
                yellowConesMsg.y(closestPointYellow.y);
                od4.send(yellowConesMsg, sampleTime, 1);

                std::vector<cv::Rect> orangeBox = findBoundingBox(contoursOrange);
                std::vector<cv::Point> orangePointsInRange;
                for(auto &box : orangeBox){
                    cv::Scalar const orange(0,165,255);
                    cv::rectangle(img, box, orange);
                    cv::Point center(box.x + box.width/2,box.y+box.height);
                      circle( img,center,5,cv::Scalar( 0, 255, 255),cv::FILLED,cv::LINE_8 );
                      cv::Point newPointOrange(center.x-(img.cols/2), img.rows-center.y);
                      orangePointsInRange.push_back(newPointOrange);
                      
                }
                int orangeMinY = 10000;
                int orangeXValue = 10000;
                for (auto &point : orangePointsInRange)
                    {
                        if (point.y < orangeMinY){
                            orangeMinY = point.y;
                            orangeXValue = point.x;
                        
                        }
                    }
                cv::Point closestPointOrange(orangeXValue,orangeMinY);
                std::cout <<"Closest point orange: " << closestPointOrange <<std::endl;
                std::cout <<" "<<std::endl;
                opendlv::logic::perception::Cones orangeConesMsg;
                orangeConesMsg.x(closestPointOrange.x);
                orangeConesMsg.y(closestPointOrange.y);
                od4.send(orangeConesMsg, sampleTime, 3);

                std::vector<cv::Rect> carBackBox = findBoundingBox(contoursCarBack);
                std::vector<double> distanceCarBack;
                std::vector<cv::Point> carBackPointsInRange;
                for(auto &box : carBackBox){
                    cv::Scalar const red(0, 0, 255);
                    cv::rectangle(img, box, red);
                    cv::Point center(box.x + box.width/2,box.y+box.height);
                    circle( img,center,5,cv::Scalar(0, 0, 255),cv::FILLED,cv::LINE_8 );
                      
                    cv::Point newPointCarBack(center.x-(img.cols/2), img.rows-center.y);
                    carBackPointsInRange.push_back(newPointCarBack);
                    
                }
                int carBackMinY = 10000;
                int carBackXValue = 10000;
                for (auto &point : carBackPointsInRange)
                    {
                        if (point.y < carBackMinY){
                            carBackMinY = point.y;
                            carBackXValue = point.x;
                        }

                    }
                cv::Point closestPointCarBack(carBackXValue,carBackMinY);
                std::cout <<"Closest point car back: " << closestPointCarBack <<std::endl;
                
                opendlv::logic::perception::Cones carBackMsg;
                carBackMsg.x(closestPointCarBack.x);
                carBackMsg.y(closestPointCarBack.y);
                od4.send(carBackMsg, sampleTime,2);
                
                // Display image.
                if (VERBOSE) {
                    cv::Mat contours(img.size(), CV_8UC3, cv::Scalar(0, 0, 0));
                    
                    cv::drawContours(contours, contoursBlue, -1, cv::Scalar(255, 0, 0), 2);
                    cv::drawContours(contours, contoursYellow, -1, cv::Scalar(0, 255, 255), 2);
                    cv::drawContours(contours, contoursCarBack, -1, cv::Scalar(0, 255, 255), 2);
                    
                    
                    cv::imshow("Img", img);
                }

                char key = (char) cv::waitKey(1);
                if (key == 'q' || key == 27)
                {
                    break;
                }
            }
        }
        retCode = 0;
    }
    return retCode;
}
