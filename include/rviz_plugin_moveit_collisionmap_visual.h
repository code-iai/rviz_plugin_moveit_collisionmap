#ifndef __RVIZ_PLUGIN_MOVEIT_COLLISIONMAP_VISUAL_H__
#define __RVIZ_PLUGIN_MOVEIT_COLLISIONMAP_VISUAL_H__


#include <list>
#include <moveit_msgs/CollisionMap.h>
#include <geometry_msgs/Vector3.h>

using namespace std;


namespace Ogre {
  class Vector3;
  class Quaternion;
}

namespace rviz {
  class Shape;
}

namespace rviz_plugin_moveit_collisionmap {
  class MoveItCollisionMapVisual {
  public:
    // Constructor. Creates the visual stuff and puts it into the
    // scene, but in an unconfigured state.
    MoveItCollisionMapVisual(Ogre::SceneManager* scene_manager,
			     Ogre::SceneNode* parent_node);
    
    // Destructor. Removes the visual stuff from the scene.
    virtual ~MoveItCollisionMapVisual();
    
    // Configure the visual to show the data in the message.
    void setMessage(const moveit_msgs::CollisionMap::ConstPtr& msg);
    
    // Set the pose of the coordinate frame the message refers to.
    // These could be done inside setMessage(), but that would require
    // calls to FrameManager and error handling inside setMessage(),
    // which doesn't seem as clean. This way ImuVisual is only
    // responsible for visualization.
    void setFramePosition(const Ogre::Vector3& position);
    void setExtents(const Ogre::Vector3& extents);
    
    // Set the color and alpha of the visual, which are user-editable
    // parameters and therefore don't come from the Imu message.
    void setColor(float r, float g, float b, float a);
    
  private:
    // The object implementing the actual arrow shape
    boost::shared_ptr<rviz::Shape> cube_;
    
    // A SceneNode whose pose is set to match the coordinate frame of
    // the Imu message header.
    Ogre::SceneNode* frame_node_;
    
    // The SceneManager, kept here only so the destructor can ask it to
    // destroy the ``frame_node_``.
    Ogre::SceneManager* scene_manager_;
  };
}


#endif /* __RVIZ_PLUGIN_MOVEIT_COLLISIONMAP_VISUAL_H__ */
