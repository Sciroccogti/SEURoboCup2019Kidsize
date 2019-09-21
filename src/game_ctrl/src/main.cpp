#include <ros/ros.h>
#include <QApplication>
#include "gamectrl.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gamectrl");
    QApplication app(argc, argv);
    std::string cfg="";
    if(argc>1)
        cfg=std::string(argv[1]);
    StartDlg sdlg(cfg);
    sdlg.exec();
    if(sdlg.redName.size()>0){
        GameCtrl foo(sdlg.redName, sdlg.blueName);
        foo.show();
        return app.exec();
    }
    return 0;
}