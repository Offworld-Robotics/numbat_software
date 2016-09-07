/* ERC Competition map publisher
 * By Nuno Das Neves
 * Date 1/9/2016 - 8/9/2016
 * Opens a geotiff map file and publishes it as an occupancyGrid
 */

#include "ERCMapPub.h"

int main (int argc, char *argv[]) {
    
    ros::init(argc, argv, "owr_erc_map_pub");
    //std::cout << "ros init'd" << std::endl;
    Astar compMap("owr_auton_pathing");
    compMap.spin();
    return 0;
}

ERCMapPub::ERCMapPub(const std::string topic) {
    getMap();
    
    mapPublisher = node.advertise<nav_msgs::OccupancyGrid>("owr_erc_map", 2, true);
    
    // TODO ????
    tfFilter.registerCallback(boost::bind(&ERCMapPub::tfCallback, this, _1));
}



// gets a transform from SLAM (between map and base_link)
void ERCMapPub::tfCallback(const nav_msgs::OccupancyGrid::ConstPtr& gridData) {
    
    tfListener.lookupTransform("map","base_link", gridData->header.stamp, transform);
   
    ROS_INFO("base_link at (%f, %f)", transform.getOrigin().getX(), transform.getOrigin().getY());
    
    // this gets the map from SLAM, which we're not doing anymore
    //makeGrid((int8_t*)gridData->data.data(), gridData->info);
    
    //TODO make sure this works
    // ??? geotiff origin? a pose in the real world of the centre of the map
    outputGrid.info.origin.position = gridData->info.origin.position;
    outputGrid.header = gridData->header;
    
}

//ros's main loop thing?
void ERCMapPub::spin() {
    //std::cout << "spinning" << std::endl;
    while(ros::ok()) {
        ros::spinOnce();
    }
}


void ERCMapPub::getMap() {
    // load image from file
    cv::Mat image;
    image = cv::imread(IMG_PATH, CV_LOAD_IMAGE_GRAYSCALE);   // Read the file
    cv::Mat bimage = image.clone();
    cv::Mat fimage = image.clone();
    
    if(!image.data)                              // Check for invalid input
    {
        ROS_ERROR ("Could not open or find %s", IMG_PATH);
        return;
    }
    
    int tot;
    int howmany = 0;
    double alpha = 1.5f;
        
    for(int c=0; c < image.cols; ++c) {
        for(int r=0; r < image.rows; ++r) {
            image.at<uchar>(r,c) = cv::saturate_cast<uchar>(alpha*(image.at<uchar>(r,c)));
            if (image.at<uchar>(r,c) > 250) image.at<uchar>(r,c) = 0;
        }
    }
    
    //blur image with cv's GaussianBlur
    for ( int i = 1; i < 21; i += 2 ) cv::GaussianBlur(image, bimage, cv::Size(i, i), 0, 0);
    
    // This gives us the gradient/ differential map. Whatever its called. High pass filter?
    for(int c=0; c < image.cols; ++c) {
        for(int r=0; r < image.rows; ++r) {
            // loop through adjacent pixels
            for(int co=-11; co <= 11; ++co) {
                for(int ro=-11; ro <= 11; ++ro) {
                    // if within a circle around our pixel,
                    if (r+ro < image.rows && c+co < image.cols
                        && r+ro >= 0 && c+co >= 0 && (ro != 0 || co != 0)
                        && sqrt((ro*ro)+(co+co)) <= 11)
                    {
                        // count em up for cases where we go over the edge (so the average is computed accurately)
                        howmany ++;
                        // add to the total
                        tot += abs(bimage.at<uchar>(r,c) - bimage.at<uchar>(r+ro,c+co));
                    }
                }
            }
            // get average and stick in new image
            fimage.at<uchar>(r,c) = cv::saturate_cast<uchar>(tot/howmany);
            // reset stuff
            howmany = 0;
            tot = 0;
        }
    }
    ROS_INFO ("made diff map");
    
    // linear filter to bump up the whites
    for(int c=0; c < image.cols; ++c) {
        for(int r=0; r < image.rows; ++r) {
            fimage.at<uchar>(r,c) = cv::saturate_cast<uchar>(1.8f*(fimage.at<uchar>(r,c)));
        }
    }
    ROS_INFO ("brightened");
    
    
    // Do the radius thing - this will surround impassable pixels with almost impassable pixels 
    // This gives a buffer zone for the rover
    for(int c=0; c < image.cols; ++c) {
        for(int r=0; r < image.rows; ++r) {
            // if the pixel is over our threshold
            if (fimage.at<uchar>(r,c) > 100) {
                // loop through adjacent pixels
                for(int co=-6; co <= 6; ++co) {
                    for(int ro=-6; ro <= 6; ++ro) {
                        // make sure its within a circle around our pixel, and lower than the threshold
                        if (r+ro < image.rows && c+co < image.cols
                            && r+ro >= 0 && c+co >= 0 && (ro != 0 || co != 0)
                            && sqrt((ro*ro)+(co+co)) <= 6 && fimage.at<uchar>(r+ro,c+co) < 100) {
                            // set to threshold
                            fimage.at<uchar>(r+ro,c+co) = 100;
                        }
                    }
                }
            }
        }
    }
    ROS_INFO ("did radius thing");
    //cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
    //cv::imshow( "Display window", fimage );                   // Show our image inside it.
    //cv::waitKey(0);                                          // Wait for a keystroke in the window
    
    // free da memory
    image.release();
    bimage.release();
    
    // resize astarGrid
    astarGrid.resize(fimage.cols);
    //set entire grid to impassable
    for(unsigned int i = 0; i < astarGrid.size(); ++i) {
        astarGrid[i].resize(fimage.rows);
        for(unsigned int j = 0; j < astarGrid[i].size(); ++j) {
            astarGrid[i][j] = IMPASS;
        }
    }
    
    
    outputGrid.info.width=fimage.cols;
    outputGrid.info.height=fimage.rows;
    // each pixel is 0.090375m
    outputGrid.info.resolution = 0.090375;
    outputGrid.info.map_load_time = ros::Time::now();
    // resize the data vector to the right size (???????)
    std::vector<int8_t> vec;
    vec.resize(fimage.rows*fimage.cols);
    outputGrid.data = vec;
    
    int i = 0;
    unsigned char value;
    // loopy loop through each pixel in greyscale
    
    for(int r=0; r < fimage.rows; r++) {
        for(int c=0; c < fimage.cols; c++) {
            // pretty sure this is the right way around
            value = (int)fimage.at < uchar>(r,c);
            //astarGrid[c][r] = value;
            // populate the grid we're going to publish
            outputGrid.data[i] = value;
            i++;
        }
    }
}