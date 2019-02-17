#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

std::vector<cv::KeyPoint> keypoints;
cv::Mat descriptors;
cv::Mat img;

bool CheckFAST(const cv::Mat& img,int x, int y){
    int width= img.cols;
    int height= img.rows;
    if(x<=4 || x>=width-4 ||y<=4 || y>=height-4){
        return false;
    }
    cv::Point2i test_p(x,y);
    int test_illu=img.at<unsigned char>(test_p);
    
    std::vector<cv::Point2i> offset(32);
    offset[0]=cv::Point2i(0,-3);
    offset[1]=cv::Point2i(1,-3);
    offset[2]=cv::Point2i(2,-2);
    offset[3]=cv::Point2i(3,-1);
    offset[4]=cv::Point2i(3,0);
    offset[5]=cv::Point2i(3,1);
    offset[6]=cv::Point2i(2,2);
    offset[7]=cv::Point2i(1,3);
    offset[8]=cv::Point2i(0,3);
    offset[9]=cv::Point2i(-1,3);
    offset[10]=cv::Point2i(-2,2);
    offset[11]=cv::Point2i(-3,1);
    offset[12]=cv::Point2i(-3,0);
    offset[13]=cv::Point2i(-3,-1);
    offset[14]=cv::Point2i(-2,-2);
    offset[15]=cv::Point2i(-1,-3);
    offset[16]=cv::Point2i(0,-3);
    offset[17]=cv::Point2i(1,-3);
    offset[18]=cv::Point2i(2,-2);
    offset[19]=cv::Point2i(3,-1);
    offset[20]=cv::Point2i(3,0);
    offset[21]=cv::Point2i(3,1);
    offset[22]=cv::Point2i(2,2);
    offset[23]=cv::Point2i(1,3);
    offset[24]=cv::Point2i(0,3);
    offset[25]=cv::Point2i(-1,3);
    offset[26]=cv::Point2i(-2,2);
    offset[27]=cv::Point2i(-3,1);
    offset[28]=cv::Point2i(-3,0);
    offset[29]=cv::Point2i(-3,-1);
    offset[30]=cv::Point2i(-2,-2);
    offset[31]=cv::Point2i(-1,-3);
    int accu_plus_count=0;
    int accu_mins_count=0;
    bool find_corner=false;
    for(int i=0; i<32 ; i++){
        int offset_illu=img.at<unsigned char>(offset[i]+test_p);
        if (test_illu>offset_illu){
            accu_plus_count++;
            if(accu_plus_count>9){
                find_corner=true;
                break;
            }
        }else{
            accu_plus_count=0;
        }
        if (test_illu<offset_illu){
            accu_mins_count++;
            if(accu_mins_count>9){
                find_corner=true;
                break;
            }
        }else{
            accu_mins_count=0;
        }
    }
    return find_corner;
}

void HarrisResponses(const cv::Mat& img, const std::vector<cv::Rect>& layerinfo,
                std::vector<cv::KeyPoint>& pts, int blockSize, float harris_k)
{
    using namespace cv;
    CV_Assert( img.type() == CV_8UC1 && blockSize*blockSize <= 2048 );

    size_t ptidx, ptsize = pts.size();

    const uchar* ptr00 = img.ptr<uchar>();
    int step = (int)(img.step/img.elemSize1());
    int r = blockSize/2;

    float scale = 1.f/((1 << 2) * blockSize * 255.f);
    float scale_sq_sq = scale * scale * scale * scale;

    AutoBuffer<int> ofsbuf(blockSize*blockSize);
    int* ofs = ofsbuf;
    for( int i = 0; i < blockSize; i++ )
        for( int j = 0; j < blockSize; j++ )
            ofs[i*blockSize + j] = (int)(i*step + j);

    for( ptidx = 0; ptidx < ptsize; ptidx++ )
    {
        int x0 = cvRound(pts[ptidx].pt.x);
        int y0 = cvRound(pts[ptidx].pt.y);
        int z = pts[ptidx].octave;

        const uchar* ptr0 = ptr00 + (y0 - r + layerinfo[z].y)*step + x0 - r + layerinfo[z].x;
        int a = 0, b = 0, c = 0;

        for( int k = 0; k < blockSize*blockSize; k++ )
        {
            const uchar* ptr = ptr0 + ofs[k];
            int Ix = (ptr[1] - ptr[-1])*2 + (ptr[-step+1] - ptr[-step-1]) + (ptr[step+1] - ptr[step-1]);
            int Iy = (ptr[step] - ptr[-step])*2 + (ptr[step-1] - ptr[-step-1]) + (ptr[step+1] - ptr[-step+1]);
            a += Ix*Ix;
            b += Iy*Iy;
            c += Ix*Iy;
        }
        pts[ptidx].response = ((float)a * b - (float)c * c -
                               harris_k * ((float)a + b) * ((float)a + b))*scale_sq_sq;
    }
}

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if  ( event == cv::EVENT_LBUTTONDOWN )
    {
        cv::KeyPoint kp;
        kp.pt.x=x;
        kp.pt.y=y;
        kp.octave=0;
        std::vector<cv::KeyPoint> pts;
        pts.push_back(kp);
        std::vector<cv::Rect> layerInfo(1);
        layerInfo[0].x=0;
        layerInfo[0].y=0;
        HarrisResponses(img, layerInfo, pts , 7, 0.04);
        bool is_corver = CheckFAST(img,x, y);
        cv::Ptr<cv::ORB> detector = cv::ORB::create();
        cv::Mat loc_descriptors;
        detector->compute( img, pts ,loc_descriptors);
        std::cout<<"("<<x<<","<<y<<") "<<is_corver<<" | "<<pts[0].response*1e9<<std::endl;
        std::cout<<loc_descriptors<<std::endl;

    }
    else if  ( event == cv::EVENT_RBUTTONDOWN )
    {
        //std::cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
    }
    else if  ( event == cv::EVENT_MBUTTONDOWN )
    {
        //std::cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
    }
    else if ( event == cv::EVENT_MOUSEMOVE )
    {
        //std::cout << "Mouse move over the window - position (" << x << ", " << y << ")" << std::endl;
    }    
}


int main(int argc, char* argv[]) {
    std::string root_addr = argv[1];
    std::vector<std::string> img_addr_list;
    for(int i=2; i<argc ; i++){
        img_addr_list.push_back(root_addr+argv[i]);
    }
    if(img_addr_list.size()<=0){
        return 0;
    }
    
    cv::namedWindow("ImageDisplay", 1);
    cv::setMouseCallback("ImageDisplay", CallBackFunc, NULL);
    
    int cur_img=0;
    float cur_scale=1;
    cv::Scalar_<unsigned char> color = cv::Scalar(0, 0, 255);
    while(true){
        img = cv::imread(img_addr_list[cur_img]);
        cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
        cv::resize(img, img, cv::Size(), cur_scale, cur_scale);
        cv::Ptr<cv::ORB> detector = cv::ORB::create();
        detector->setNLevels(1);
        
        detector->detectAndCompute(img, cv::noArray(), keypoints, descriptors);
        std::cout<<"img id: "<<cur_img<<" | kp count: "<<keypoints.size()<<std::endl;
        cv::Mat img_display;
        cv::cvtColor(img, img_display, cv::COLOR_GRAY2BGRA);
        for(int i=0; i<keypoints.size(); i++){
            img_display.at<cv::Scalar_<unsigned char>>(keypoints[i].pt)=color;
        }
        
        cv::imshow("ImageDisplay", img_display);
        int re_key = cv::waitKey(-1);
        if((unsigned char)re_key=='a' && cur_img>0){
            cur_img--;
        }else if((unsigned char)re_key=='d' && cur_img<img_addr_list.size()-1){
            cur_img++;
        }else if((unsigned char)re_key=='w'){
            cur_scale=cur_scale-0.1;
        }else if((unsigned char)re_key=='s'){
            cur_scale=cur_scale+0.1;
        }else if((unsigned char)re_key=='q'){
            break;
        }
    }
}