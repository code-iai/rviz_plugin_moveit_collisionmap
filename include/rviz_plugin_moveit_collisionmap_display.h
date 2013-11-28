#ifndef __RVIZ_PLUGIN_MOVEIT_COLLISIONMAP_DISPLAY_H__
#define __RVIZ_PLUGIN_MOVEIT_COLLISIONMAP_DISPLAY_H__


#include <boost/circular_buffer.hpp>

#include <moveit_msgs/CollisionMap.h>
#include <rviz/message_filter_display.h>


namespace Ogre {
  class SceneNode;
}

namespace rviz {
  class ColorProperty;
  class FloatProperty;
  class IntProperty;
}

namespace rviz_plugin_moveit_collisionmap {
  class MoveItCollisionMapVisual;
  
  class MoveItCollisionMapDisplay : public rviz::MessageFilterDisplay<moveit_msgs::CollisionMap> {
    Q_OBJECT
    private Q_SLOTS:
      void updateColorAndAlpha();
      void updateHistoryLength();
      
    private:
      void processMessage(const moveit_msgs::CollisionMap::ConstPtr& msg);

      boost::circular_buffer<boost::shared_ptr<MoveItCollisionMapVisual> > visuals_;

      // User-editable property variables.
      rviz::ColorProperty* color_property_;
      rviz::FloatProperty* alpha_property_;
      rviz::IntProperty* history_length_property_;

    protected:
      virtual void onInitialize();
      virtual void reset();

    public:
      MoveItCollisionMapDisplay();
      virtual ~MoveItCollisionMapDisplay();
  };
}


#endif /* __RVIZ_PLUGIN_MOVEIT_COLLISIONMAP_DISPLAY_H__ */
