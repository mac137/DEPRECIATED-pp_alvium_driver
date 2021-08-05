#include <nodelet/nodelet.h>
#include "pp_alvium_driver/mono_camera.h"

namespace avt_vimba_camera
{

    class MonoCameraNodelet : public nodelet::Nodelet
    {
        public:
            virtual void onInit();
            virtual ~MonoCameraNodelet();
        private:
            MonoCamera* camera_;
    };

}
