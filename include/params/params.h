namespace params{
	
	template<typename T>
	T load( std::string name, ros::NodeHandle &nh){
		T param;
		if (nh.hasParam( name )){
		    nh.getParam( name, param);
		    return param;
		} else {
		    std::cout << "Param " << name << " does not exist." << std::endl;
		    exit(0);
		}
    }
    
}