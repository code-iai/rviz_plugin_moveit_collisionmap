#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>

#include "rviz_plugin_moveit_collisionmap_visual.h"

#include "rviz_plugin_moveit_collisionmap_display.h"


namespace rviz_plugin_moveit_collisionmap {
  MoveItCollisionMapDisplay::MoveItCollisionMapDisplay() {
    color_property_ = new rviz::ColorProperty("Color", QColor(204, 51, 204),
					      "Base color to draw the cubes.",
					      this, SLOT(updateColorAndAlpha()));
    
    alpha_property_ = new rviz::FloatProperty("Alpha", 1.0,
					      "0 is fully transparent, 1.0 is fully opaque.",
					      this, SLOT(updateColorAndAlpha()));
    
    history_length_property_ = new rviz::IntProperty("History Length", 5000,
						     "Number of cubes to display at the same time.",
						     this, SLOT(updateHistoryLength()));
    history_length_property_->setMin(1);
    history_length_property_->setMax(100000);
  }
  
  void MoveItCollisionMapDisplay::onInitialize() {
    MFDClass::onInitialize();
    updateHistoryLength();
  }
  
  MoveItCollisionMapDisplay::~MoveItCollisionMapDisplay() {
  }
  
  // Clear the visuals by deleting their objects.
  void MoveItCollisionMapDisplay::reset() {
    MFDClass::reset();
    visuals_.clear();
  }
  
  // Set the current color and alpha values for each visual.
  void MoveItCollisionMapDisplay::updateColorAndAlpha() {
    float alpha = alpha_property_->getFloat();
    Ogre::ColourValue color = color_property_->getOgreColor();

    for(size_t i = 0; i < visuals_.size(); i++) {
      visuals_[i]->setColor(color.r, color.g, color.b, alpha);
    }
  }
  
  // Set the number of past visuals to show.
  void MoveItCollisionMapDisplay::updateHistoryLength() {
    visuals_.rset_capacity(history_length_property_->getInt());
  }
  
  // This is our callback to handle an incoming message.
  void MoveItCollisionMapDisplay::processMessage(const moveit_msgs::CollisionMap::ConstPtr& msg) {
    // Here we call the rviz::FrameManager to get the transform from the
    // fixed frame to the frame in the header of this Imu message. If
    // it fails, we can't do anything else so we return.
    Ogre::Quaternion orientation;
    Ogre::Vector3 position;
    if(!context_->getFrameManager()->getTransform(msg->header.frame_id,
						  msg->header.stamp,
						  position, orientation)) {
      ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
		msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
      return;
    }
    
    // We are keeping a circular buffer of visual pointers. This gets
    // the next one, or creates and stores it if the buffer is not full
    visuals_.clear();
    boost::shared_ptr<MoveItCollisionMapVisual> visual;
    
    vector<moveit_msgs::OrientedBoundingBox> vecBoxes = msg->boxes;
    for(vector<moveit_msgs::OrientedBoundingBox>::iterator itBox = vecBoxes.begin();
    	itBox != vecBoxes.end();
    	itBox++) {
      moveit_msgs::OrientedBoundingBox bxBox = *itBox;
      const geometry_msgs::Point& ptPosition = bxBox.pose.position;
      const geometry_msgs::Point32& ptExtents = bxBox.extents;
      
      Ogre::Vector3 vecPosition(ptPosition.x, ptPosition.y, ptPosition.z);
      Ogre::Vector3 vecExtents(ptExtents.x - 0.01, ptExtents.y - 0.01, ptExtents.z - 0.01);
      
      //visual->setFramePosition(vecPosition);
      //cube_->setPosition(vecPosition);
      //cube_->setScale(vecExtents);
      
      if(visuals_.full()) {
	visual = visuals_.front();
      } else {
	visual.reset(new MoveItCollisionMapVisual(context_->getSceneManager(), scene_node_));
      }
      
      // Now set or update the contents of the chosen visual.
      visual->setMessage(msg);
      visual->setFramePosition(vecPosition);
      visual->setExtents(vecExtents);
      
      float alpha = alpha_property_->getFloat();
      Ogre::ColourValue color = color_property_->getOgreColor();
      visual->setColor(color.r, color.g, color.b, alpha);
      
      // And send it to the end of the circular buffer
      
      // NOTE(winkler): Check here if this cube is already in the buffer.
      visuals_.push_back(visual);
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_moveit_collisionmap::MoveItCollisionMapDisplay, rviz::Display)
