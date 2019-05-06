#include "System.h"
#import "RecordViewController.h"
#include "common_header.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <rosbag/view.h>
#include <cv_bridge/cv_bridge.h>
#include "read_write_data_lib/read_write.h"
#include <bag_tool/extract_bag.h>
#include "imu_tools.h"
#import "IOS_visualization.h"
#include "optimizer_tool/optimizer_tool.h"
#include "create_desc_index.h"


@implementation RecordViewController

- (void)viewDidLoad
{
    [super viewDidLoad];
    sessionQueue = dispatch_queue_create( "session queue", DISPATCH_QUEUE_SERIAL );
    quene =[[NSOperationQueue alloc] init];
    quene.maxConcurrentOperationCount=1;
    motionManager = [[CMMotionManager alloc] init];
    
    session = [[AVCaptureSession alloc] init];
    NSError *error = nil;
    [session beginConfiguration];
    session.sessionPreset =AVCaptureSessionPreset640x480;
    videoDevice = [AVCaptureDevice defaultDeviceWithDeviceType:AVCaptureDeviceTypeBuiltInWideAngleCamera mediaType:AVMediaTypeVideo position:AVCaptureDevicePositionUnspecified];
    AVCaptureDeviceInput *videoDeviceInput = [AVCaptureDeviceInput deviceInputWithDevice:videoDevice error:&error];
    if ( ! videoDeviceInput ) {
        NSLog( @"Could not create video device input: %@", error );
        [session commitConfiguration];
        return;
    }
    if ( [session canAddInput:videoDeviceInput] ) {
        [session addInput:videoDeviceInput];
        videoDeviceInput = videoDeviceInput;
    }
    else {
        NSLog( @"Could not add video device input to the session" );
        [session commitConfiguration];
        return;
    }
    [videoDevice setActiveVideoMinFrameDuration:CMTimeMake(1, 10)];
    if ( [videoDevice lockForConfiguration:&error] ) {
        videoDevice.exposureMode=AVCaptureExposureModeLocked;
        [videoDevice setExposureModeCustomWithDuration:CMTimeMakeWithSeconds( 0.001, 1000*1000*1000 ) ISO:200 completionHandler:nil];
    }
    [videoDevice unlockForConfiguration];
    
    video_output = [[AVCaptureVideoDataOutput alloc] init];
    NSDictionary *newSettings = @{ (NSString *)kCVPixelBufferPixelFormatTypeKey : @(kCVPixelFormatType_32BGRA) };
    video_output.videoSettings = newSettings;
    [video_output setAlwaysDiscardsLateVideoFrames:YES];
    if ([session canAddOutput:video_output]) {
        [session addOutput:video_output];
    }else {
        NSLog(@"add output wrong!!!");
    }
    
    [video_output setSampleBufferDelegate:self queue:sessionQueue];
    
    [session commitConfiguration];
    img_count=0;
    dele_bag= [[BagListDelegate alloc] init];
    dele_map= [[BagListDelegate alloc] init];
    self.bag_list_ui.delegate = dele_bag;
    self.bag_list_ui.dataSource = dele_bag;
    self.map_list_ui.delegate = dele_map;
    self.map_list_ui.dataSource = dele_map;
    is_locating=false;
}

void interDouble(double v1, double v2, double t1, double t2, double& v3_out, double t3){
    v3_out=v1+(v2-v1)*(t3-t1)/(t2-t1);
}

- (void) processIMU_gyro{
    int last_acc_id=-1;
    int last_gyro_id=-1;
    int g_size=gyros.size();
    int a_size=acces.size();
    for(int i=0;i<gyros.size();i++){
        for(int j=0;j<a_size-1;j++){
            if(gyros[i][3]>acces[j][3] && gyros[i][3]<=acces[j+1][3]){
                double x,y,z;
                interDouble(acces[j][0], acces[j+1][0], acces[j][3], acces[j+1][3], x, gyros[i][3]);
                interDouble(acces[j][1], acces[j+1][1], acces[j][3], acces[j+1][3], y, gyros[i][3]);
                interDouble(acces[j][2], acces[j+1][2], acces[j][3], acces[j+1][3], z, gyros[i][3]);
                last_acc_id=j;
                last_gyro_id=i;
                sensor_msgs::Imu msg;
                msg.linear_acceleration.x=x;
                msg.linear_acceleration.y=y;
                msg.linear_acceleration.z=z;
                msg.angular_velocity.x=gyros[i][0];
                msg.angular_velocity.y=gyros[i][1];
                msg.angular_velocity.z=gyros[i][2];
                static int imu_data_seq=0;
                msg.header.seq=imu_data_seq;
                msg.header.stamp= ros::Time(gyros[i][3]);
                msg.header.frame_id="map";
                dispatch_async(sessionQueue, ^{
                    if (is_recording_bag){
                        if(bag_ptr->isOpen()){
                            NSDate * t1 = [NSDate date];
                            NSTimeInterval now = [t1 timeIntervalSince1970];
                            bag_ptr->write("imu", ros::Time(now), msg);
                        }
                    }
                    if(is_locating){
                        Eigen::Vector3d Accl(msg.linear_acceleration.x,
                                             msg.linear_acceleration.y,
                                             msg.linear_acceleration.z);
                        Eigen::Vector3d Gyro(msg.angular_velocity.x,
                                             msg.angular_velocity.y,
                                             msg.angular_velocity.z);
                        double timestamp = msg.header.stamp.toSec();
                        //NSLog(@"imu: %@", [NSThread currentThread]);
                        localizer->AddIMU( timestamp,Accl,Gyro);
                        //std::cout<<"add imu"<<std::endl;
                    }
                });
                
                break;
            }
        }
    }
    if(last_acc_id>0){
        if(last_acc_id-1<acces.size()){
            acces.erase(acces.begin(), acces.begin()+last_acc_id);
        }else{
            NSLog(@"test overflow");
        }
    }
    if(last_gyro_id>=0){
        if(last_gyro_id<gyros.size()){
            gyros.erase(gyros.begin(), gyros.begin()+last_gyro_id+1);
        }else{
            NSLog(@"test overflow");
        }
    }
}

- (UIImage *) imageFromSampleBuffer:(CMSampleBufferRef) sampleBuffer
{
    CVImageBufferRef imageBuffer = CMSampleBufferGetImageBuffer(sampleBuffer);
    CVPixelBufferLockBaseAddress(imageBuffer, 0);
    void *baseAddress = CVPixelBufferGetBaseAddress(imageBuffer);
    size_t bytesPerRow = CVPixelBufferGetBytesPerRow(imageBuffer);
    size_t width = CVPixelBufferGetWidth(imageBuffer);
    size_t height = CVPixelBufferGetHeight(imageBuffer);
    CGColorSpaceRef colorSpace = CGColorSpaceCreateDeviceRGB();
    CGContextRef context = CGBitmapContextCreate(baseAddress, width, height, 8,
                                                 bytesPerRow, colorSpace, kCGBitmapByteOrder32Little | kCGImageAlphaPremultipliedFirst);
    CGImageRef quartzImage = CGBitmapContextCreateImage(context);
    CVPixelBufferUnlockBaseAddress(imageBuffer,0);
    CGContextRelease(context);
    CGColorSpaceRelease(colorSpace);
    UIImage *image = [UIImage imageWithCGImage:quartzImage];
    CGImageRelease(quartzImage);
    return image;
}

- (void)update_baglist{
    NSArray *dirPaths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSArray *directoryContent = [[NSFileManager defaultManager] contentsOfDirectoryAtPath:[dirPaths objectAtIndex:0] error:NULL];
    NSMutableArray *directoryContent_bag=[[NSMutableArray alloc] init];
    NSMutableArray *directoryContent_map=[[NSMutableArray alloc] init];
    
    for (int count = 0; count < (int)[directoryContent count]; count++)
    {
        NSString * full_str = [[dirPaths objectAtIndex:0] stringByAppendingPathComponent:[directoryContent objectAtIndex:count]];
        bool isDirectory;
        bool exist_b;
        exist_b = [[NSFileManager defaultManager] fileExistsAtPath:full_str isDirectory:&isDirectory];

        if(isDirectory){
            [directoryContent_map addObject:[directoryContent objectAtIndex:count]];
        }else{
            [directoryContent_bag addObject:[directoryContent objectAtIndex:count]];
        }
    }
    
    dele_map.file_list =directoryContent_map;
    dele_bag.file_list =directoryContent_bag;
    
    if([directoryContent_bag count]>0){
        dele_bag.sel_filename=[directoryContent_bag objectAtIndex:0];
        [self.bag_list_ui selectRow:0 inComponent:0 animated:NO];
    }
    if([directoryContent_map count]>0){
        dele_map.sel_filename=[directoryContent_map objectAtIndex:0];
        [self.map_list_ui selectRow:0 inComponent:0 animated:NO];
    }
    
    [self.bag_list_ui reloadAllComponents];
    [self.map_list_ui reloadAllComponents];
}

-(void) do_slam: (std::string) voc_str mycam_str:(std::string) mycam_str bag_str:(std::string) bag_str full_file_name:(std::string) full_file_name Rwwc:(Eigen::Matrix3d) Rwwc{
    ORB_SLAM2::System sys(voc_str, mycam_str);
    rosbag::Bag bag;
    bag.open(bag_str,rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back("img");
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    int slam_img_count=0;
    rosbag::View::iterator it= view.begin();
    for(;it!=view.end();it++){
        rosbag::MessageInstance m =*it;
        sensor_msgs::CompressedImagePtr simg = m.instantiate<sensor_msgs::CompressedImage>();
        if(simg!=NULL){
            cv_bridge::CvImagePtr cv_ptr;
            try{
                std::stringstream ss;
                ss<<"img_"<<slam_img_count<<".jpg";
                cv_ptr = cv_bridge::toCvCopy(simg, "mono8");
                slam_img_count++;
                sys.TrackMonocular(cv_ptr->image, simg->header.stamp.toSec(), ss.str());
                std::vector<Eigen::Vector3d> pcs;
                sys.getPC(pcs);
                std::vector<Eigen::Vector3d> posis;
                std::vector<Eigen::Quaterniond> quas;
                sys.getTraj(posis, quas);
                float reproject_err_t;
                int match_count_t;
                int mp_count_t;
                int kf_count_t;
                sys.getDebugImg(img_display, reproject_err_t, match_count_t, mp_count_t, kf_count_t);
                if(!img_display.empty()){
                    [self.frameDelegate showFrame: img_display];
                    [self.frameDelegate showCurInfo: reproject_err_t match_count: match_count_t mp_count: mp_count_t kf_count: kf_count_t];
                    //std::cout<<"posis.size(): "<<posis.size()<<std::endl;
                    if(posis.size()>0){
                        if(last_kf_count<posis.size()){
                            for(int i=0; i<posis.size(); i++){
                                posis[i]= Rwwc*posis[i];
                            }
                            [_sceneDelegate showTraj: posis];
                        }
                    }
                    last_kf_count=posis.size();
                }
            }catch (cv_bridge::Exception& e){
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
        }
    }
    sys.saveResult(full_file_name);
}

- (IBAction)start_opti:(id)sender {
    
    if((int)[dele_map.file_list count]>0){
        dispatch_async( sessionQueue, ^{
            NSArray *dirPaths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
            NSString *full_addr = [[dirPaths objectAtIndex:0] stringByAppendingPathComponent:dele_map.sel_filename];
            char *full_addr_char = (char*)[full_addr cStringUsingEncoding:[NSString defaultCStringEncoding]];
            std::string full_addr_std(full_addr_char);
            NSBundle* myBundle = [NSBundle mainBundle];
            NSString* word_proj_str;
            word_proj_str = [myBundle pathForResource:@"words_projmat" ofType:@"dat"];
            NSFileManager *fileManager= [NSFileManager defaultManager];
            char *docsPath;
            docsPath = (char*)[full_addr cStringUsingEncoding:[NSString defaultCStringEncoding]];
            std::string full_file_name(docsPath);
            std::string words_projmat_std=full_file_name+"/words_projmat.dat";
            NSString * words_projmat_ns = [NSString stringWithFormat:@"%s",words_projmat_std.c_str()];
            [fileManager copyItemAtPath:word_proj_str toPath:words_projmat_ns error:nil];
            NSLog(word_proj_str);
            NSLog(words_projmat_ns);
            OptimizerTool::optimize_imu(full_addr_std);
            DescIndex::create_desc_index(full_addr_std);
        } );
    }
}

- (IBAction)show_map:(id)sender {
    if((int)[dele_map.file_list count]>0){
        NSArray *dirPaths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
        NSString *full_addr = [[dirPaths objectAtIndex:0] stringByAppendingPathComponent:dele_map.sel_filename];
        char *full_addr_char = (char*)[full_addr cStringUsingEncoding:[NSString defaultCStringEncoding]];
        std::string full_addr_std(full_addr_char);
        [IOSVis showMPs: full_addr_std+"/posi_alin.txt" sceneDelegate: _sceneDelegate];
        [IOSVis showTraj: full_addr_std+"/traj_alin.txt" sceneDelegate: _sceneDelegate];
    }
}

- (void) start_slam: (NSString *) bag_name
{
    NSBundle* myBundle = [NSBundle mainBundle];
    NSString* mycam_str;
    mycam_str = [myBundle pathForResource:@"iphone" ofType:@"yaml"];
    NSString* voc_str;
    voc_str = [myBundle pathForResource:@"small_voc_no_rot" ofType:@"yml"];
    NSString* cam_config_str;
    cam_config_str = [myBundle pathForResource:@"camera_config" ofType:@"txt"];

    NSString* bag_str=nil;
    NSArray *dirPaths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSArray *directoryContent = [[NSFileManager defaultManager] contentsOfDirectoryAtPath:[dirPaths objectAtIndex:0] error:NULL];
    for (int count = 0; count < (int)[directoryContent count]; count++)
    {
        NSError *error = nil;
        bag_str = [[dirPaths objectAtIndex:0] stringByAppendingPathComponent:[directoryContent objectAtIndex:count]];
        if([bag_name isEqualToString:[directoryContent objectAtIndex:count]]==YES){
            break;
        }
    }
    NSLog(bag_str);
    NSDate *date = [NSDate date];
    NSDateFormatter *formatter = [[NSDateFormatter alloc] init];
    [formatter setDateFormat:@"MM-dd-HH-mm-ss"];
    NSString *timeString = [formatter stringFromDate:date];
    NSString *string1 = [NSString stringWithFormat:@"%@",timeString];
    NSString *full_addr = [[dirPaths objectAtIndex:0] stringByAppendingPathComponent:string1];
    NSFileManager *fileManager= [NSFileManager defaultManager];
    if(![fileManager createDirectoryAtPath:full_addr withIntermediateDirectories:YES attributes:nil error:NULL]){
        NSLog(@"Error: Create folder failed %@", full_addr);
        return;
    }
    std::string full_file_name=[full_addr UTF8String];
    std::string cam_config_std=full_file_name+"/camera_config.txt";
    NSString * cam_config_ns_desc = [NSString stringWithFormat:@"%s",cam_config_std.c_str()];
    [fileManager copyItemAtPath:cam_config_str toPath:cam_config_ns_desc error:nil];
    extract_img_imu(full_file_name, [bag_str UTF8String], "img", "imu");
    Eigen::Matrix3d cam_inter;
    Eigen::Vector4d cam_distort;
    Eigen::Matrix4d Tbc;
    CHAMO::read_cam_info(full_file_name+"/camera_config.txt", cam_inter, cam_distort, Tbc);
    std::string imu_addr=full_file_name+"/imu.txt";
    std::vector<Eigen::Matrix<double, 7, 1>> imu_datas_raw;
    CHAMO::read_imu_data(imu_addr, imu_datas_raw);
    Eigen::Matrix3d Rwi;
    if(imu_datas_raw.size()>0){
        Eigen::Vector3d first_acce=imu_datas_raw[0].block(0,0,3,1);
        Rwi = orb_slam::calRotMFromGravity(first_acce);
    }
    [self do_slam: [voc_str UTF8String] mycam_str:[mycam_str UTF8String] bag_str:[bag_str UTF8String] full_file_name:full_file_name Rwwc: Rwi.transpose()*Tbc.block(0,0,3,3)];
}

- (void)captureOutput:(AVCaptureOutput *)captureOutput didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer fromConnection:(AVCaptureConnection *)connection {
    CMTime timestamp = CMSampleBufferGetPresentationTimeStamp(sampleBuffer);
    double sync_sensor_time = (double)timestamp.value/(double)timestamp.timescale;
    double time_sec = (double)timestamp.value/(double)timestamp.timescale;
    UIImage *image = [self imageFromSampleBuffer:sampleBuffer];
    cv::Mat img_cv = [mm_Try cvMatFromUIImage:image];
    [self.frameDelegate showFrame: img_cv];
    sensor_msgs::CompressedImage img_ros_img;
    std::vector<unsigned char> binaryBuffer_;
    cv::imencode(".jpg", img_cv, binaryBuffer_);
    img_ros_img.data=binaryBuffer_;
    img_ros_img.header.seq=img_count;
    img_ros_img.header.stamp= ros::Time(sync_sensor_time);
    img_ros_img.format="jpeg";
    
//    sensor_msgs::ImagePtr img_ros_img;
//    cv_bridge::CvImage img_cvbridge;
//    img_cvbridge.image=img_cv;
//    img_ros_img=img_cvbridge.toImageMsg();
//    img_ros_img->encoding="bgra8";
//    img_ros_img->header.seq=img_count;
//    img_ros_img->header.stamp= ros::Time(sync_sensor_time);
    dispatch_async( sessionQueue, ^{
        if(is_recording_bag){
            if(bag_ptr->isOpen()){
                NSDate * t1 = [NSDate date];
                NSTimeInterval now = [t1 timeIntervalSince1970];
                bag_ptr->write("img", ros::Time(now), img_ros_img);
            }
        }
        if(is_locating){
            double timestamp = img_ros_img.header.stamp.toSec();
            cv::Mat img_grey;
            cv::cvtColor(img_cv, img_grey, cv::COLOR_BGR2GRAY);
            localizer->AddImage(timestamp,0,img_grey);
            Eigen::Vector3d Pos;
            Eigen::Vector3d Vel;
            Eigen::Quaterniond Ori;
            bool re =localizer->QueryPose(-1,  Pos,  Vel,  Ori);
            if (re){
                loc_posi_re.push_back(Pos);
                [_sceneDelegate showTraj: loc_posi_re];
            }
            
            //NSLog(@"img: %@", [NSThread currentThread]);
            //std::cout<<"add img"<<std::endl;
        }
    });
    img_count++;
}

- (IBAction)exit_btn:(id)sender {
    [self dismissViewControllerAnimated:false completion: nil];
}

- (void)viewWillDisappear:(BOOL)animated{
}
- (IBAction)start_record:(id)sender {
    if(!is_recording_bag){
        dispatch_async( sessionQueue, ^{
            NSArray *dirPaths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
            NSDate *date = [NSDate date];
            NSDateFormatter *formatter = [[NSDateFormatter alloc] init];
            [formatter setDateFormat:@"MM-dd-HH-mm-ss"];
            NSString *timeString = [formatter stringFromDate:date];
            NSString *string1 = [NSString stringWithFormat:@"%@.bag",timeString];
            NSString *full_addr = [[dirPaths objectAtIndex:0] stringByAppendingPathComponent:string1];
            char *docsPath;
            docsPath = (char*)[full_addr cStringUsingEncoding:[NSString defaultCStringEncoding]];
            std::string full_file_name(docsPath);
            std::cout<<full_file_name<<std::endl;
            bag_ptr.reset(new rosbag::Bag());
            bag_ptr->open(full_file_name.c_str(), rosbag::bagmode::Write);
            is_recording_bag=true;
        });
        //[self update_baglist];
        [sender setTitle:@"Stop" forState:UIControlStateNormal];
    }else{
        is_recording_bag=false;
        dispatch_async( sessionQueue, ^{
            bag_ptr->close();
            NSLog(@"close the bag");
        });
        [sender setTitle:@"Record" forState:UIControlStateNormal];
        [self update_baglist];
    }
    
}

- (IBAction)start_sensor:(id)sender {
    if(session.running){
        [session stopRunning];
        [sender setTitle:@"Start Sensor" forState:UIControlStateNormal];
    }else{
        [session startRunning];
        [sender setTitle:@"Stop" forState:UIControlStateNormal];
    }
    
    if(motionManager.accelerometerActive){
        [motionManager stopAccelerometerUpdates];
    }else{
        if (motionManager.accelerometerAvailable){
            motionManager.accelerometerUpdateInterval =0.01;
            [motionManager
             startAccelerometerUpdatesToQueue:quene
             withHandler:
             ^(CMAccelerometerData *data, NSError *error){
                 std::vector<double> imu;
                 imu.resize(5);
                 imu[0]=-data.acceleration.x*9.8;
                 imu[1]=-data.acceleration.y*9.8;
                 imu[2]=-data.acceleration.z*9.8;
                 imu[3]=data.timestamp;
                 imu[4]=0;
                 acces.push_back(imu);
             }];
        }
    }
    
    if(motionManager.gyroActive){
        [motionManager stopGyroUpdates];
    }else{
        if (motionManager.gyroAvailable){
            motionManager.gyroUpdateInterval =0.01;
            [motionManager
             startGyroUpdatesToQueue:quene
             withHandler:
             ^(CMGyroData *data, NSError *error){
                 std::vector<double> imu;
                 imu.resize(5);
                 imu[0]=data.rotationRate.x;
                 imu[1]=data.rotationRate.y;
                 imu[2]=data.rotationRate.z;
                 imu[3]=data.timestamp;
                 imu[4]=0;
                 gyros.push_back(imu);
                 [self processIMU_gyro];
             }];
        }
    }
}
- (IBAction)start_mapping:(id)sender {
    if((int)[dele_bag.file_list count]>0){
        dispatch_async( sessionQueue, ^{
             [self start_slam:dele_bag.sel_filename];
        } );
    }
}
- (IBAction)load_map:(id)sender {
    NSBundle* myBundle = [NSBundle mainBundle];
    NSString* myloc_str;
    myloc_str = [myBundle pathForResource:@"rovio_default_config" ofType:@"info"];
    NSArray *dirPaths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *full_addr = [[dirPaths objectAtIndex:0] stringByAppendingPathComponent:dele_map.sel_filename];
    char *full_addr_char = (char*)[full_addr cStringUsingEncoding:[NSString defaultCStringEncoding]];
    std::string full_addr_std(full_addr_char);
    localizer.reset(new wayz::ChamoLoc);
    localizer->StartLocalization([myloc_str UTF8String]);
    localizer->AddMap(full_addr_std);
}
- (IBAction)locate:(id)sender {
    if(is_locating==true){
        is_locating=false;
    }else{
        is_locating=true;
    }
    
}

- (void)viewDidAppear:(BOOL)animated{
    [self update_baglist];
}

- (IBAction)go_to_frame_page:(id)sender {
    [self dismissViewControllerAnimated:false completion: nil];
}


@end
