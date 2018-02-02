#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <gazebo/transport/transport.hh>
# include <gazebo/gui/gui.hh>
#endif

namespace gazebo
{
    class GAZEBO_VISIBLE POILabeling : public GUIPlugin
    {
      Q_OBJECT
	
		public:
      		POILabeling();
      		virtual ~POILabeling();

		private:
      		/// \brief Callback for pre-render event.
      		void Update();

      		/// \brief Callback for model update event.
      		/// \param[in] _msg Incoming message.
     		void OnModelUpdate(const msgs::Model& msg);

      		/// \brief Map with names of all models in the scene and a flag indicating
      		/// whether they've been processed or not.
      		std::map<std::string, bool> models;

      		/// \brief All the event connections.
      		std::vector<event::ConnectionPtr> connections;

      		/// \brief Mutex to protect variables
     		std::mutex mutex;
    };
}
