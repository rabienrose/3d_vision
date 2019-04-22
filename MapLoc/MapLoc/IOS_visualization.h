#include "common_header.h"
#include "std_msgs/String.h"
#include "delegate_header.h"
#import "UIListDelegate.h"

@interface IOSVis: NSObject
    + (void)showMPs: (std::string) mp_addr sceneDelegate: (id<SceneInfoDelegate>) sceneDelegate;
    + (void)showTraj: (std::string) traj_addr  sceneDelegate: (id<SceneInfoDelegate>) sceneDelegate;
@end
