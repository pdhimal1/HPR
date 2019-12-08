This docker compose only works with the custom pcl docker image created by the author.

The docker image itself is big (24GB) and is hard to distribute.

One only needs the docker image to be able to compile and run the program. If the application can 
be compiled and run differently, there should be no need for docker image.

The custom docker image was built using: 
   - ubuntu
   - opengl
   - pcl 1.9
   - cmake 3.15
   - open3d

Of these following are required to compile the project:
   - Cmake 3.15
   - c++14
   - Qt5
   - VTK
   - PCL 1.8

To run the docker containers: 
   - docker-compose up --build

To access the running container:
   - docker exec -it pcl bash

To run the executable: 
   - `./HPR -pcdFile ../../data/data-master/tutorial/ism_test_michael.pcd`
   - `./HPR -pcdFile ../../data/data-master/tutorial/ism_test_michael.pcd`

