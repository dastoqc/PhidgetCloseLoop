#include <opencv2/highgui/highgui.hpp>

int track(cv::Mat plotImage, bool debugwin);
int findParam ( std::string param,int argc, char *argv[] );
bool readArguments ( int argc,char **argv );