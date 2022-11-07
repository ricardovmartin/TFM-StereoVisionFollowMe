#include "prueba/prueba.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "prueba");
  ros::NodeHandle n;

  prueba nodo_prueba(n);

  //ros::spin();
  nodo_prueba.main();
  return 0;
}


