#include <vector>

struct AmmoType;
struct DroneConfig;

class IConfigLoader {
public:
    virtual ~IConfigLoader() = default;
    virtual bool load() = 0;

    virtual DroneConfig getConfig() = 0;
    virtual const std::vector<AmmoType>& getAmmoParams() = 0;
};
