/* ERC Competition map publisher
 * By Nuno Das Neves
 * Date 1/9/2016 - 8/9/2016
 * Opens a geotiff map file and publishes it as an occupancyGrid
 * Uses UTM coordinates from geotiff as origin; ie you need a UTM transform for it to work
 */

#include "ERCMapPub.h"

int main (int argc, char *argv[]) {
    
    ros::init(argc, argv, "owr_erc_map");
    ERCMapPub compMap("owr_auton_pathing");
    compMap.spin();
    
    return 0;
}

ERCMapPub::ERCMapPub(const std::string topic) {
    selectedPath = -1;
    
    stringSubscriber = node.subscribe<std_msgs::String>("erc_path_string", 2, &ERCMapPub::stringCallback, this);
    
    getGeoData();
    getMap();
    mapPublisher = node.advertise<nav_msgs::OccupancyGrid>("erc_map", 2, true);
    pathPublisher = node.advertise<nav_msgs::Path>("owr_auton_pathing", 2, true);
}

void ERCMapPub::stringCallback(const std_msgs::String::ConstPtr& whichString) {
    if (whichString->data[0] < 'A' || whichString->data[0] > 'D') {
        selectedPath = -1;
    } else {
        selectedPath = whichString->data[0];
    }
}

void ERCMapPub::publish() {
    //
    outputGrid.header.frame_id = "world";
    outputGrid.header.stamp = ros::Time::now();
    
    // from converted geotiff data...
    outputGrid.info.resolution = 0.093f;
    outputGrid.info.origin.position.x = 571435.31f;
    outputGrid.info.origin.position.y = 5663420.69f;
    ROS_INFO ("Publishing a map");
    mapPublisher.publish(outputGrid);
    
    //
    outputGrid.header.frame_id = "world";
    outputPath.header.stamp = ros::Time::now();
    // WE NEED 4 paths, start->A, A->B, B->C, C->D
    std::vector<geometry_msgs::PoseStamped> pathData;
    geometry_msgs::PoseStamped temp;
    temp.header.frame_id="world";
    temp.header.stamp=ros::Time::now();
    
    // look! woah
    switch (selectedPath) {
    // START 5663390 571458
        case 'A':
            // - 5663397 571476
            // A 5663393 571479
            pathData.resize(2);
            temp.pose.position.y = 5663397;
            temp.pose.position.x = 571476;
            pathData[0]=temp;
            temp.pose.position.y = 5663393;
            temp.pose.position.x = 571479;
            pathData[1]=temp;
            break;
        case 'B':
            // - 5663388 571484
            // - 5663383 571484
            //!B 5663380 571480
            // B 5663381 571480
            pathData.resize(3);
            temp.pose.position.y = 5663388;
            temp.pose.position.x = 571484;
            pathData[0]=temp;
            temp.pose.position.y = 5663383;
            temp.pose.position.x = 571484;
            pathData[1]=temp;
            temp.pose.position.y = 5663380;
            temp.pose.position.x = 571480;
            pathData[2]=temp;
            break;
        case 'C':
            // - 5663380 571475
            // - 5663378 571471
            // C 5663378 571468
            pathData.resize(3);
            temp.pose.position.y = 5663380;
            temp.pose.position.x = 571475;
            pathData[0]=temp;
            temp.pose.position.y = 5663378;
            temp.pose.position.x = 571471;
            pathData[1]=temp;
            temp.pose.position.y = 5663378;
            temp.pose.position.x = 571468;
            pathData[2]=temp;
            break;
        case 'D':
            // - 5663382 571465
            // - 5663384 571465
            //!D 5663386 571465
            // D 5663386 571463
            pathData.resize(3);
            temp.pose.position.y = 5663382;
            temp.pose.position.x = 571465;
            pathData[0]=temp;
            temp.pose.position.y = 5663384;
            temp.pose.position.x = 571465;
            pathData[1]=temp;
            temp.pose.position.y = 5663386;
            temp.pose.position.x = 571465;
            break;
        default:
            ROS_ERROR("No path selected");
            return;
    }
    // No.
    // X 5663383 571470
    outputPath.poses = pathData;
    ROS_INFO ("Publishing a path");
    pathPublisher.publish(outputPath);
    
}

void ERCMapPub::getGeoData() {
    /*
    GDALDataset *poDataset;
    GDALAllRegister();
    poDataset = (GDALDataset *)GDALOpen(IMG_PATH_GEO, GA_ReadOnly);
    if(poDataset == NULL) {
        ROS_ERROR ("Unknown format \"%s\"",IMG_PATH_GEO);
        return;
    }
    double adfGeoTransform[6];
    // printf( "Driver: %s/%s\n\n",
    // poDataset->GetDriver()->GetDescription(), 
    // poDataset->GetDriver()->GetMetadataItem( GDAL_DMD_LONGNAME ) );
    // printf( "Size is %dx%dx%d\n\n", 
    // poDataset->GetRasterXSize(), poDataset->GetRasterYSize(),
    // poDataset->GetRasterCount());
    // if(poDataset->GetProjectionRef() != NULL)
    // printf("Projection is `%s'\n\n", poDataset->GetProjectionRef());
    if(poDataset->GetGeoTransform(adfGeoTransform) == CE_None) {
        ROS_INFO("Origin = (%.6f,%.6f)",
        adfGeoTransform[0], adfGeoTransform[3]);
        ROS_INFO("Pixel Size = (%.6f,%.6f)",
        adfGeoTransform[1], adfGeoTransform[5]);
        
        // get our pixel resolution and UTM origin coordinates
        outputGrid.info.resolution = std::abs(adfGeoTransform[1]);
        outputGrid.info.origin.position.x = adfGeoTransform[0];
        outputGrid.info.origin.position.y = adfGeoTransform[3];
        /*
    } else {
        ROS_ERROR ("Could not get geo data from \"%s\"",IMG_PATH_GEO);
    }
    */
    // hopefully this all works lel
}

void ERCMapPub::getMap() {
    
    // load image from file
    cv::Mat image;
    image = cv::imread(IMG_PATH_OCC, CV_LOAD_IMAGE_GRAYSCALE);   // Read the file
    cv::Mat bimage = image.clone();
    cv::Mat fimage = image.clone();
    
    if(!image.data)                              // Check for invalid input
    {
        ROS_ERROR ("Could not open or find %s", IMG_PATH_OCC);
        return;
    }
    
    // IMAGE PROCESSING FOR HEIGHT MAP:
    /*
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
                        tot += std::abs(bimage.at<uchar>(r,c) - bimage.at<uchar>(r+ro,c+co));
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
    */
    // TESTING only:
    //cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
    //cv::imshow( "Display window", fimage );                   // Show our image inside it.
    //cv::waitKey(0);                                          // Wait for a keystroke in the window
    
    // free da memory
    image.release();
    bimage.release();
    
    outputGrid.info.width=fimage.cols;
    outputGrid.info.height=fimage.rows;
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
            // TODO replace magic number. This just maps [0,255] to [0,100]
            value /= 2.55f;
            //astarGrid[c][r] = value;
            // populate the grid we're going to publish
            outputGrid.data[i] = value;
            i++;
        }
    }
}

void ERCMapPub::spin() {
    ros::Rate rater(0.2);
    while(ros::ok()) {
        ros::spinOnce();
        publish();
        rater.sleep();
    }
}
