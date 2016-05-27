#include "peg_sensor/peg_world_wrapper/peg_world_wrapper.h"
#include "optitrack_rviz/type_conversion.h"


Peg_world_wrapper::Peg_world_wrapper(ros::NodeHandle &nh,
                                     SOCKET_TYPE socket_type,
                                     bool bVisualise,
                                     const std::string& node_name,
                                     const std::string& path_sensor_model,
                                     const std::string& fixed_frame,
                                     const std::string &peg_link_name,
                                     const std::string table_link_name,
                                     const std::string socket_link_name,
                                     const std::string socket_link_box_name)
    :fixed_frame(fixed_frame),
      socket_type(socket_type),
      bVisualise(bVisualise),
     table_link_name(table_link_name),
     socket_link_name(socket_link_name),
     socket_link_box_name(socket_link_box_name)
{

    // Initialise the table wall
    initialise_table_wall(table_link_name);
    initialise_socket(socket_link_name,socket_link_box_name);

    world_publisher = std::shared_ptr<ww::Publisher>(new ww::Publisher( node_name + "/visualization_marker",&nh,&world_wrapper));
    world_publisher->init(fixed_frame);
    world_publisher->update_position();

    peg_sensor_model = std::shared_ptr<Peg_sensor_model>(new Peg_sensor_model(path_sensor_model,fixed_frame,peg_link_name,world_wrapper.wrapped_objects));


    if(bVisualise){

        v_surf.resize(1);
        v_edge.resize(1);
        v_corner.resize(1);

        vis_proj_sur.reset(new opti_rviz::Vis_points(nh,"proj_surf"));
        vis_proj_sur->scale = 0.005;
        vis_proj_sur->alpha = 1;
        vis_proj_sur->r     = 1;
        vis_proj_sur->g     = 0;
        vis_proj_sur->b     = 0;
        vis_proj_sur->initialise(fixed_frame,v_surf);

        vis_proj_edge.reset(new opti_rviz::Vis_points(nh,"proj_edge"));
        vis_proj_edge->scale = 0.005;
        vis_proj_edge->alpha = 1;
        vis_proj_edge->r     = 0;
        vis_proj_edge->g     = 0;
        vis_proj_edge->b     = 1;
        vis_proj_edge->initialise(fixed_frame,v_edge);

        /// Visualise socket

        vis_socket = std::shared_ptr<obj::Vis_socket>(new  obj::Vis_socket(nh,world_wrapper.wrapped_objects.wsocket));
        vis_socket->initialise(25,"world",0.001);

        /// Peg model (Cartesian points);

        vis_points = std::shared_ptr<opti_rviz::Vis_points>( new  opti_rviz::Vis_points(nh,"peg_model"));
        vis_points->scale = 0.005;
        vis_points->initialise(fixed_frame,peg_sensor_model->get_model());
    }

}

void Peg_world_wrapper::initialise_table_wall(const std::string table_link_name){

    tf::StampedTransform transform;
    opti_rviz::Listener::get_tf_once(fixed_frame,table_link_name,transform);

    tf::Vector3  wall_origin = transform.getOrigin();

    geo::fCVec3 origin       = {{(float)wall_origin.x(),(float)wall_origin.y(),(float)wall_origin.z()}};//{{0,0,-0.02/2}};
    float      dx            = 0.1;
    origin(0)                = origin(0) - 0.005; // half centimeter
    origin(0)                = origin(0)-dx/2;
    geo::fCVec3 dim          = {{static_cast<float>(0.02+dx),0.8,0.4}};
    geo::fCVec3 orientation  = {{0,0,0}};

    wbox = wobj::WBox(table_link_name,dim,origin,orientation);
    world_wrapper.wrapped_objects.push_back_box(&wbox);
}

void Peg_world_wrapper::initialise_socket(const std::string& socket_link_name,const std::string& socket_box_link_name){

    tf::StampedTransform transform;
    opti_rviz::Listener::get_tf_once(fixed_frame,socket_link_name,transform);
    opti_rviz::Listener::print(transform);

    /// add a socket
    tf::Vector3 origin = transform.getOrigin();
    tf::Vector3 rpy(M_PI/2,0,M_PI/2);

    std::cout<< "Initialise_socket" << std::endl;

    if(socket_type == SOCKET_TYPE::ONE){
        std::cout<< "   SOCKET ONE" << std::endl;

        socket_one = obj::Socket_one(socket_link_name,socket_box_link_name,origin,rpy,1);
        world_wrapper.wrapped_objects.push_back_box(&(socket_one.wbox));
        world_wrapper.wrapped_objects.push_back_socket(socket_one.wsocket);

        world_wrapper.wrapped_objects.push_back_box(&(socket_one.hole_wboxes[0]));
        world_wrapper.wrapped_objects.push_back_box(&(socket_one.hole_wboxes[1]));
        world_wrapper.wrapped_objects.push_back_box(&(socket_one.hole_wboxes[2]));

        world_wrapper.wrapped_objects.push_back_box(&(socket_one.edge_wboxes[0]));
        world_wrapper.wrapped_objects.push_back_box(&(socket_one.edge_wboxes[1]));
        world_wrapper.wrapped_objects.push_back_box(&(socket_one.edge_wboxes[2]));
        world_wrapper.wrapped_objects.push_back_box(&(socket_one.edge_wboxes[3]));


    }else if(socket_type == SOCKET_TYPE::TWO){
        std::cout<< "   SOCKET TWO" << std::endl;

        socket_two = obj::Socket_two(socket_link_name,socket_box_link_name,origin,rpy,1);
        world_wrapper.wrapped_objects.push_back_box(&(socket_two.wbox));
        world_wrapper.wrapped_objects.push_back_socket(socket_two.wsocket);

        world_wrapper.wrapped_objects.push_back_box(&(socket_two.hole_wboxes[0]));
        world_wrapper.wrapped_objects.push_back_box(&(socket_two.hole_wboxes[1]));
        world_wrapper.wrapped_objects.push_back_box(&(socket_two.hole_wboxes[2]));

        world_wrapper.wrapped_objects.push_back_box(&(socket_two.edge_wboxes[0]));
        world_wrapper.wrapped_objects.push_back_box(&(socket_two.edge_wboxes[1]));
        world_wrapper.wrapped_objects.push_back_box(&(socket_two.edge_wboxes[2]));
        world_wrapper.wrapped_objects.push_back_box(&(socket_two.edge_wboxes[3]));

    }else if(socket_type == SOCKET_TYPE::THREE){
        std::cout<< "   SOCKET THREE" << std::endl;

        socket_three = obj::Socket_three(socket_link_name,socket_box_link_name,origin,rpy,1);
        world_wrapper.wrapped_objects.push_back_box(&(socket_three.wbox));
        world_wrapper.wrapped_objects.push_back_socket(socket_three.wsocket);

        world_wrapper.wrapped_objects.push_back_box(&(socket_three.hole_wboxes[0]));
        world_wrapper.wrapped_objects.push_back_box(&(socket_three.hole_wboxes[1]));
        world_wrapper.wrapped_objects.push_back_box(&(socket_three.hole_wboxes[2]));

        world_wrapper.wrapped_objects.push_back_box(&(socket_three.edge_wboxes[0]));
        world_wrapper.wrapped_objects.push_back_box(&(socket_three.edge_wboxes[1]));
        world_wrapper.wrapped_objects.push_back_box(&(socket_three.edge_wboxes[2]));
        world_wrapper.wrapped_objects.push_back_box(&(socket_three.edge_wboxes[3]));
    }


}

void Peg_world_wrapper::update(){

    world_publisher->update_position();


    if(bVisualise){

        peg_sensor_model->update();

        vis_points->update(peg_sensor_model->get_model());

        opti_rviz::type_conv::vec2tf(peg_sensor_model->get_closet_point(SURFACE),v_surf[0]);
        opti_rviz::type_conv::vec2tf(peg_sensor_model->get_closet_point(EDGE),v_edge[0]);

        vis_proj_sur->update(v_surf);
        vis_proj_sur->publish();

        vis_proj_edge->update(v_edge);
        vis_proj_edge->publish();

        vis_points->publish();
        vis_socket->publish();

        world_publisher->publish();
    }
}

ww::World_wrapper& Peg_world_wrapper::get_world_wrapper(){
    return world_wrapper;
}

wobj::WrapObject& Peg_world_wrapper::get_wrapped_objects(){
    return world_wrapper.wrapped_objects;
}

void Peg_world_wrapper::initialise_urdf(const std::string& table_urdfs, const std::string &fixed_frame){
/*
    world_wrapper.loadURDF(table_urdfs);
    world_wrapper.initialise_origin_orientation(world_wrapper,fixed_frame);
    geo::fCVec3 T = {{0,0,-0.02}};
    for(std::size_t i = 0; i < world_wrapper.wrapped_objects.wboxes.size();i++){
        world_wrapper.wrapped_objects.wboxes[i].transform(T);
    }*/

}


