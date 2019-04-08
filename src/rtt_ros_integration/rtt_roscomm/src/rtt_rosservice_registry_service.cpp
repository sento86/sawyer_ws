
#include <rtt/RTT.hpp>
#include <rtt/plugin/ServicePlugin.hpp>
#include <rtt/internal/GlobalService.hpp>

#include <rtt_roscomm/rtt_rosservice_registry_service.h>
#include <rtt_roscomm/rtt_rosservice_proxy.h>

ROSServiceRegistryServicePtr ROSServiceRegistryService::s_instance_;

ROSServiceRegistryServicePtr ROSServiceRegistryService::Instance()
{
  if (!s_instance_) {
    s_instance_.reset(new ROSServiceRegistryService(0));
    RTT::internal::GlobalService::Instance()->addService(s_instance_);
  }
  return s_instance_;
}

void ROSServiceRegistryService::Release()
{
  s_instance_.reset();
}

ROSServiceRegistryService::ROSServiceRegistryService(RTT::TaskContext* owner)
  : Service("rosservice_registry", owner)
{
  this->doc("Global RTT Service for registering ROS service types.");
  this->addOperation("registerServiceFactory", &ROSServiceRegistryService::registerServiceFactory, this, RTT::ClientThread);
  this->addOperation("hasServiceFactory", &ROSServiceRegistryService::hasServiceFactory, this, RTT::ClientThread);
  this->addOperation("getServiceFactory", &ROSServiceRegistryService::getServiceFactory, this, RTT::ClientThread);
  this->addOperation("listSrvs", &ROSServiceRegistryService::listSrvs, this, RTT::ClientThread);
}

/** \brief Register a ROS service proxy factory
 *
 * This enables the ROSServiceRegistryService to construct ROS service clients and
 * servers from a string name.
 */
bool ROSServiceRegistryService::registerServiceFactory(ROSServiceProxyFactoryBase* factory)
{
  RTT::os::MutexLock lock(factory_lock_);
  if(factory == NULL) {
    RTT::log(RTT::Error) << "Failed to register ROS service factory: NULL pointer given." << RTT::endlog();
    return false;
  }

  const std::string &ros_service_type = factory->getType();

  // Check if the factory type has yet to be registered
  if(factories_.find(ros_service_type) == factories_.end()) {
    // Store a new factory
    factories_[ros_service_type] = boost::shared_ptr<ROSServiceProxyFactoryBase>(factory);
  } else {
    // Reset the existing factory
    factories_[ros_service_type].reset(factory);
  }

  RTT::log(RTT::Info) << "Successfully registered ROS service factory for \"" << ros_service_type << "\"." << RTT::endlog();

  return true;
}

bool ROSServiceRegistryService::hasServiceFactory(const std::string &service_type)
{
  RTT::os::MutexLock lock(factory_lock_);
  return factories_.find(service_type) != factories_.end();
}

ROSServiceProxyFactoryBase* ROSServiceRegistryService::getServiceFactory(const std::string &service_type)
{
  RTT::os::MutexLock lock(factory_lock_);
  if(factories_.find(service_type) != factories_.end()) {
    return factories_[service_type].get();
  }

  RTT::log(RTT::Error)<<"Service type \""<<service_type<<"\" has not been registered with the rosservice_registry service."<<RTT::endlog();

  return NULL;
}

void ROSServiceRegistryService::listSrvs()
{
  RTT::os::MutexLock lock(factory_lock_);

  RTT::log(RTT::Info) << "Available ROS .srv types:" << RTT::endlog();
  for(std::map<std::string, boost::shared_ptr<ROSServiceProxyFactoryBase> >::const_iterator it = factories_.begin();
      it != factories_.end();
      ++it)
  {
    RTT::log(RTT::Info) << " -- " << it->first << RTT::endlog();
  }
}

std::map<std::string, boost::shared_ptr<ROSServiceProxyFactoryBase> > ROSServiceRegistryService::factories_;
RTT::os::MutexRecursive ROSServiceRegistryService::factory_lock_;

void loadROSServiceRegistryService()
{
  ROSServiceRegistryService::Instance();
}

using namespace RTT;
extern "C" {
  bool loadRTTPlugin(RTT::TaskContext* c){
    if (c != 0) return false;
    loadROSServiceRegistryService();
    return true;
  }
  std::string getRTTPluginName (){
    return "rosservice_registry";
  }
  std::string getRTTTargetName (){
    return OROCOS_TARGET_NAME;
  }
}
