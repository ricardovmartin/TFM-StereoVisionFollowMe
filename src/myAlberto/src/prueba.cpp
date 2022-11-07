#include "prueba/prueba.h"
using namespace cv;
using namespace std;
using namespace std::chrono;
ofstream fich;
ofstream fich1;

// Imagen-> Ancho: 1280; Alto: 720
void prueba::depthCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  // Get a pointer to the depth values casting the data
  // pointer to floating point
  depths = (float*)(&msg->data[0]); 
  
  // Image coordinates of the center pixel
  int u = msg->width / 2;
  int v = msg->height / 2;

  // Linear index of the center pixel
  int centerIdx = u + msg->width * v;

  float suma_depth = 0;
  media_depth = 0;

  if(flag_tracking){
    cv_image_depth = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1); // Depth map
    mat_image_depth = cv_image_depth->image;
    mat_image_depth = mat_image_depth/30.0; // Normalize depth values (Max depth 30 m)
    Mat uchar_mat_image_depth;
    mat_image_depth.convertTo(uchar_mat_image_depth, CV_8UC1, 255, 0); // Grayscale

    Mat color_img;
    cvtColor(uchar_mat_image_depth, color_img, CV_GRAY2RGB); // Grayscale to RGB
    Mat image_color = color_img;

    if((bbox.y+bbox.height)>720) // In case box is higher than max values, limit it
      bbox.height = 719-bbox.y;

    image_color = image_color(bbox); // Depth image cropped

    Mat samples(image_color.rows*image_color.cols, 3, CV_32F); // kmeans data should be 32F
    for( int y = 0; y < image_color.rows; y++ )
        for( int x = 0; x < image_color.cols; x++ )
          for( int z = 0; z < 3; z++)
            samples.at<float>(y + x*image_color.rows, z) = image_color.at<Vec3b>(y,x)[z];

    Mat labels, centers;

    kmeans(samples, 3, labels, TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 10000, 0.0001), 5, KMEANS_PP_CENTERS, centers); // Kmeans clustering

    unsigned int* cluster = (unsigned int *)calloc(5, sizeof(int));
    for(unsigned int i=0; i<3; i++)
      cluster[i] = 0;
    
    Mat new_image(image_color.size(), image_color.type());
    for(int y = 0; y < image_color.rows; y++) 
      for(int x = 0; x < image_color.cols; x++){
        int cluster_idx = labels.at<int>(y + x*image_color.rows,0);
        new_image.at<Vec3b>(y,x)[0] = centers.at<float>(cluster_idx, 0);
        new_image.at<Vec3b>(y,x)[1] = centers.at<float>(cluster_idx, 1);
        new_image.at<Vec3b>(y,x)[2] = centers.at<float>(cluster_idx, 2);
        cluster[cluster_idx]++;
      }

    unsigned int cluster_person; // Most popular group belongs to person
    if(cluster[0]>cluster[1] && cluster[0]>cluster[2])
      cluster_person = 0;
    else if(cluster[1]>cluster[0] && cluster[1]>cluster[2])
      cluster_person = 1;
    else if(cluster[2]>cluster[0] && cluster[2]>cluster[1])
      cluster_person = 2;

    unsigned int cont_depth = 0;
    float suma_profundidad = 0;
    unsigned int suma_x = 0;
    unsigned int suma_y = 0;

    for(int y = 0; y < image_color.rows; y++) 
      for(int x = 0; x < image_color.cols; x++){
        int cluster_idx = labels.at<int>(y + x*image_color.rows,0);
        if(cluster_idx == cluster_person){ // If pixel belongs to person cluster, add it
          mat_image.at<Vec3b>(bbox.y+y,bbox.x+x)[0] = 0;
          mat_image.at<Vec3b>(bbox.y+y,bbox.x+x)[1] = 0;
          mat_image.at<Vec3b>(bbox.y+y,bbox.x+x)[2] = 0;
          pos_persona = (bbox.x+x) + width * (bbox.y+y);
          if(!isnan(depths[pos_persona]) && depths[pos_persona]<30){
            posiciones_x.push_back(x+bbox.x);
            posiciones_y.push_back(y+bbox.y);
            suma_profundidad += depths[pos_persona];
            cont_depth++;
          }
        }  
      }

    cont_depth = 0;

    flag_tracking = false;
    flag_cloud = true; // Enable point cloud callback

  }
}

void prueba::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{

  cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  mat_image = cv_image->image; // OpenCV image

}

void prueba::cloudCallback(const sensor_msgs::PointCloud2Ptr& msg)
{
  if(flag_cloud){
    sensor_msgs::PointCloud2 my_pcl2 = *msg;
    sensor_msgs::PointCloud my_pcl;
    sensor_msgs::convertPointCloud2ToPointCloud(my_pcl2, my_pcl);
    
    float media_depth_cloud, media_angle, varianza;
    float suma_depth = 0; 
    float suma_angle = 0;
    float suma_z = 0;
    int suma_x = 0;
    float suma_y = 0;
    unsigned int cont_cloud = 0;
    std::vector <float> v_profundidad;
    std::vector <float> v_angulo;

    // Depth and orientation calculation
    for(unsigned int pos=0; pos<posiciones_x.size(); pos++){
      pos_persona = posiciones_x.at(pos) + width * posiciones_y.at(pos);
      if(!isnan(my_pcl.points[pos_persona].x) && !isnan(depths[pos_persona]) && my_pcl.points[pos_persona].x < 30.0 && abs(my_pcl.points[pos_persona].y)<2){
        suma_depth += my_pcl.points[pos_persona].x;
        suma_angle += my_pcl.points[pos_persona].y;
        suma_z += my_pcl.points[pos_persona].z;
        cont_cloud++;
      }
      suma_x += posiciones_x[pos];
      suma_y += posiciones_y[pos];
    }  

    float media_x = suma_x/posiciones_x.size();
    float media_y = suma_y/posiciones_y.size();

    media_depth_cloud= suma_depth/cont_cloud;
    media_angle = suma_angle/cont_cloud;
    double media_z = suma_z/cont_cloud;
    
    if(media_x<width/2 && media_angle < 0.0){ // From right person view they are positive
      media_angle = media_angle*(-1.0);
    }

    float angle = atan(media_angle/media_depth_cloud);
    posiciones_x.clear();
    posiciones_y.clear();

    // KALMAN FILTER LOCATION ESTIMATION
    ros::Time current_time;
    current_time = ros::Time::now();
    // Publish odometry message
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    // Set the position
    odom.pose.pose.position.x = (double)media_depth_cloud; // x coordinate is depth
    odom.pose.pose.position.y = (double)media_angle; // y coordinate horizontal
    odom.pose.pose.position.z = media_z;
    odom.pose.covariance[0] = 0.4; // Error 40 cm
    odom.pose.covariance[7] = 0.4;
    odom.pose.covariance[14] = 0.4;

    odom_pub.publish(odom);

    flag_cloud = false;
  }
}

void prueba::ekfCallback(const nav_msgs::Odometry& odom_msg){
  nav_msgs::Odometry msg = odom_msg;
  if(flag_cloud){
    double x = odom_msg.pose.pose.position.x;
    double y = odom_msg.pose.pose.position.y;
    ROS_INFO("Coordenada x: %g", x);
    ROS_INFO("Coordenada y: %g", y);

    double angle = atan(y/x)*180/3.141592;
    ROS_INFO("Angulo: %g", angle);

    std_msgs::String msg_depth;
    msg_depth.data = to_string(x);
    depth_pub.publish(msg_depth);
    ROS_INFO_STREAM(msg_depth);

    std_msgs::String msg_angle;
    msg_angle.data = to_string(angle);
    angle_pub.publish(msg_angle);
    ROS_INFO_STREAM(angle);

  }
}

int prueba::main(){
  unsigned int first_recog = 0;
  Mat prev_image, image, prev_image_front;
  int x, y, y_height, x_width;
  Mat img_erode;
  Rect2d bbox_front;
  bool ok, ok_front;
  flag_tracking = false;
  flag_cloud = false;
  bool what_tracker = false; // False for back tracker (default), true for front tracker
  bool flag_capture = false; // To have 2 different images for recognisition while tracking

  while (ros::ok())
  {
    // INITAL BOX DEFINITION
    if(first_recog == 0){
      sleep(3);
    }
    
    else if(first_recog == 1){
      ROS_INFO("First image back recognition...");
      prev_image = mat_image;

      ROS_INFO("Move a little bit.");
      sleep(5);    
    }
    else if(first_recog == 2){
        // INITIAL PERSON RECOGNITION
        ROS_INFO("Second image back recognition...");
        image = mat_image;
        Mat fgmask;

        pBackSub->apply(prev_image, fgmask); // Background substraction
        pBackSub->apply(image, fgmask);

        img_erode = fgmask.clone();

        vector<vector<Point> > contours;
        findContours(img_erode, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        Mat mask= Mat::zeros(fgmask.rows,fgmask.cols, CV_8UC1);
        if(contours.size() == 0)
          ROS_INFO("No objects detected");
        else{
          ROS_INFO("Objects detected");
          RNG rng(0xFFFFFFFF);
          int area, dist_centro = img_erode.cols;
          int pos_centro=0;
          Scalar color;
          for(size_t i=0; i<contours.size(); i++){
            color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256)); // Random color
            Rect r= boundingRect(contours[i]);
            area = r.area();
            if(area>50000){
              if(abs(img_erode.cols/2-r.x) < dist_centro){
                pos_centro = i;
                dist_centro = abs(img_erode.cols/2-r.x);
              }
            }
          }

        drawContours(mask, contours, pos_centro, color); // Person region after first recognition

        // BOX DEFINING PERSON  
        Rect br = boundingRect(contours[pos_centro]);
        x = br.x; // br.height;
        y = br.y; // br.width;
        y_height = y+br.height;
        x_width = x+br.width;

        Mat new_image = image.clone(); // Removing static person region
        for(int i=0; i<new_image.cols; i++){
          for(int j=0; j<new_image.rows; j++){
            if(i>x && i<x_width && j>y && j<y_height){
              new_image.at<Vec3b>(j,i)[0]=0;
              new_image.at<Vec3b>(j,i)[1]=0;
              new_image.at<Vec3b>(j,i)[2]=0;
            }
          }
        }

        ROS_INFO("Back recognition done");
        Rect2d bbox(x, y, br.width, br.height); // Bounding box that belongs to the person
        rectangle(image, bbox, Scalar( 255, 0, 0 ), 2, 1 ); // Display bounding bow
        tracker->init(image,bbox); // Initialize the tracker with a known bounding box that surrounds the target
        ok = false;
        ROS_INFO("Posicion x back: %d. x end: %d",x, x+br.width);
        ROS_INFO("Turn around");
        sleep(5);
      }
    }
    else if (first_recog == 3){
      ROS_INFO("First image front recognition...");
      prev_image_front = mat_image;
      ROS_INFO("Move a little bit");
      sleep(5);
    }
    else if(first_recog == 4){
      // INITIAL PERSON RECOGNITION
      ROS_INFO("Second image front recognition...");
      Mat image_front = mat_image;
      
      Mat fgmask_front;

      pBackSub->apply(prev_image_front, fgmask_front);
      pBackSub->apply(image_front, fgmask_front);

      Mat img_erode;
      img_erode = fgmask_front.clone();

      vector<vector<Point> > contours;
      findContours(img_erode, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
      Mat mask= Mat::zeros(fgmask_front.rows,fgmask_front.cols, CV_8UC1);
      if(contours.size() == 0)
        ROS_INFO("No objects detected");
      else{
        ROS_INFO("Objects detected");
        RNG rng(0xFFFFFFFF);
        int area, dist_centro = img_erode.cols;
        int pos_centro=0;
        Scalar color;
        for(size_t i=0; i<contours.size(); i++){
          color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256)); // Color aleatorio
          Rect r= boundingRect(contours[i]);
          area = r.area();
          if(area>50000){
            if(abs(img_erode.cols/2-r.x) < dist_centro){
              pos_centro = i;
              dist_centro = abs(img_erode.cols/2-r.x);
            }
          }
        }

        drawContours(mask, contours, pos_centro, color);

        // BOX DEFINING PERSON  
        Rect br = boundingRect(contours[pos_centro]);
        x = br.x;// br.height;
        y = br.y;// br.width;
        y_height = y+br.height;
        x_width = x+br.width;

        Mat new_image = image.clone();// Remove region static person
        for(int i=0; i<new_image.cols; i++){
          for(int j=0; j<new_image.rows; j++){
            if(i>x && i<x_width && j>y && j<y_height){
              new_image.at<Vec3b>(j,i)[0]=0;
              new_image.at<Vec3b>(j,i)[1]=0;
              new_image.at<Vec3b>(j,i)[2]=0;
            }
          }
        }
        ROS_INFO("Front recognition done");
        Rect2d bbox_front(x, y, br.width, br.height); // Bounding box that belongs to the person
        rectangle(image, bbox_front, Scalar( 255, 0, 0 ), 2, 1 ); // Display bounding bow
        trackerFront->init(image,bbox_front); // Initialize the tracker with a known bounding box that surrounded the target
        ok_front = false;
        ROS_INFO("Posicion x front: %d",x);
        sleep(5);
      }
    }
  else if(first_recog > 4){       
      ROS_INFO("Tracking");
      image = mat_image;
      //sleep(1);
      
      if(!what_tracker){ // Back tracker
          ok = tracker->update(image,bbox);
          std_msgs::String mensaje;
          
          if(ok){
            rectangle(image, bbox, Scalar( 255, 0, 0 ), 2, 1 ); // Tracking success : Draw the tracked object
            flag_tracking = true;
            ROS_INFO("OK");
          }
          else{
            ROS_INFO("FAIL");
            ROS_INFO("FAIL! Turn around");
            what_tracker = true; // Front tracker
            sleep(5);
          }
        }
      else{
        ROS_INFO("Front tracker working");
        ok = trackerFront->update(image,bbox); // Updated with the last person position
        if(ok){
          rectangle(image, bbox, Scalar( 255, 0, 0 ), 2, 1 ); // Tracking success : Draw the tracked object
          flag_tracking = true;
          ROS_INFO("OK");
        }    
        else{
          ROS_INFO("FAIL! Turn around");
          what_tracker = false; // Back tracker
          sleep(5);
        }
      }*
  }     
    ros::spinOnce();
    first_recog++;
  }
}

prueba::prueba(ros::NodeHandle n): n_(n) {
  subDepth = n_.subscribe("zed_node/depth/depth_registered", 10, &prueba::depthCallback,this);
  subImage = n_.subscribe("zed_node/left/image_rect_color", 10, &prueba::imageCallback,this);
  subCloud = n_.subscribe("zed_node/point_cloud/cloud_registered", 10, &prueba::cloudCallback,this);
  subEKF = n_.subscribe("/odometry/filtered", 10, &prueba::ekfCallback, this);

  odom_pub = n_.advertise<nav_msgs::Odometry>("odom", 50);
  depth_pub = n_.advertise<std_msgs::String>("depth", 50);
  angle_pub = n_.advertise<std_msgs::String>("angle", 50);

  tracker = cv::TrackerKCF::create(); // Back tracker
  pBackSub = cv::createBackgroundSubtractorMOG2();

}
prueba::~prueba(){

}
