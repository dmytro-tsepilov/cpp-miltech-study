struct Target;
// ============ ITargetProvider Interface ============

class ITargetProvider {
public:
    virtual ~ITargetProvider() = default;
    virtual bool load() = 0;
    virtual int getTargetCount() = 0;
    virtual int getTimeSteps() = 0;
    virtual Target *getTarget(int index) = 0;
};
