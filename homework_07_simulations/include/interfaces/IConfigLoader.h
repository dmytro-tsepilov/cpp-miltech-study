#include "drone/DroneConfig.h"

class IConfigLoader {
public:
    virtual ~IConfigLoader() = default;
    virtual bool load() = 0;

    virtual DroneConfig getConfig() = 0;
    virtual AmmoType **getAmmoParams(int &ammoCount) = 0;
};
