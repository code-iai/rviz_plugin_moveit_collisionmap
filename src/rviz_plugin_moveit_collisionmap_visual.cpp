#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/shape.h>

#include "rviz_plugin_moveit_collisionmap_visual.h"


namespace rviz_plugin_moveit_collisionmap {
  MoveItCollisionMapVisual::MoveItCollisionMapVisual(Ogre::SceneManager* scene_manager,
						     Ogre::SceneNode* parent_node) {
    scene_manager_ = scene_manager;
    frame_node_ = parent_node->createChildSceneNode();
    cube_.reset(new rviz::Shape(rviz::Shape::Cube, scene_manager_, frame_node_));
    //acceleration_arrow_.reset(new rviz::Arrow(scene_manager_, frame_node_));
  }
  
  MoveItCollisionMapVisual::~MoveItCollisionMapVisual() {
    scene_manager_->destroySceneNode(frame_node_);
  }
  
  void MoveItCollisionMapVisual::setMessage(const moveit_msgs::CollisionMap::ConstPtr& msg) {
    vector<moveit_msgs::OrientedBoundingBox> vecBoxes = msg->boxes;
    
    /*for(vector<moveit_msgs::OrientedBoundingBox>::iterator itBox = vecBoxes.begin();
    	itBox != vecBoxes.end();
    	itBox++) {
      moveit_msgs::OrientedBoundingBox bxBox = *itBox;
      const geometry_msgs::Point& ptPosition = bxBox.pose.position;
      const geometry_msgs::Point32& ptExtents = bxBox.extents;
      
      Ogre::Vector3 vecPosition(ptPosition.x, ptPosition.y, ptPosition.z);
      Ogre::Vector3 vecExtents(ptExtents.x, ptExtents.y, ptExtents.z);
      
      cube_->setPosition(vecPosition);
      cube_->setScale(vecExtents);
      }*/
    
    // // Convert the geometry_msgs::Vector3 to an Ogre::Vector3.
    // Ogre::Vector3 acc(a.x, a.y, a.z);

    // // Find the magnitude of the acceleration vector.
    // float length = acc.length();

    // // Scale the arrow's thickness in each dimension along with its length.
    // Ogre::Vector3 scale(length, length, length);
    // acceleration_arrow_->setScale(scale);

    // // Set the orientation of the arrow to match the direction of the
    // // acceleration vector.
    // acceleration_arrow_->setDirection(acc);
  }
  
  // Position and orientation are passed through to the SceneNode.
  void MoveItCollisionMapVisual::setFramePosition(const Ogre::Vector3& position) {
    frame_node_->setPosition(position);
  }
  
  void MoveItCollisionMapVisual::setExtents(const Ogre::Vector3& extents) {
    //frame_node_->setScale(extents);
    cube_->setScale(extents);
  }
  
  // Color is passed through to the Arrow object.
  void MoveItCollisionMapVisual::setColor(float r, float g, float b, float a) {
    cube_->setColor(r, g, b, a);
  }
}
