struct SimStep;
// ============ IResultWriter Interface ============

class IResultWriter {
public:
    virtual ~IResultWriter() = default;
    virtual bool write(const SimStep* steps, const int &stepCount) = 0;
};
