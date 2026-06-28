struct SimStep;
#include <vector>

// ============ IResultWriter Interface ============

class IResultWriter {
public:
    virtual ~IResultWriter() = default;
    virtual bool write(const std::vector<SimStep>& steps) = 0;
};
