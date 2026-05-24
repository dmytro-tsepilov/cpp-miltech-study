#pragma once

#include <string>
// Вимикаємо попередження про тавтологічні порівняння перед інклудом json.hpp
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtautological-overlap-compare"

#include "json.hpp"
// Повертаємо попередження назад для решти коду
#pragma GCC diagnostic pop

#include "drone/DroneConfig.h"

struct SimStep {
	Coord pos;          	// позиція дрона
	float direction;    	// напрямок (рад)
	int8_t state;        	// стан автомата (0-4)
	int   targetIdx;    	// індекс поточної цілі
	Coord dropPoint;    	// точка скиду (куди летить дрон)
	Coord aimPoint;     	// куди впаде бомба (якщо скинути зараз)
	Coord predictedTarget;  // прогнозована позиція цілі
};

// ============ IResultWriter Interface ============

class IResultWriter {
public:
    virtual ~IResultWriter() = default;
    virtual bool write(SimStep* steps, int stepCount) = 0;
};

// ============ JsonResultWriter ============

class JsonResultWriter : public IResultWriter {
private:
    std::string filename;
    std::string folderPath;

public:
    JsonResultWriter(const std::string &folderPath = "", const std::string &filename = "simulation.json") {
        this->filename = filename;
        this->folderPath = folderPath;
    }

    bool write(SimStep* steps, int stepCount) override;
    void setFolderPath(const std::string folderPath = "");
    void setFilename(const std::string &filename);
};

// ============ ApiResultWriter ============

class ApiResultWriter : public IResultWriter {
private:
    std::string apiUrl;
    std::string authToken;

public:
    ApiResultWriter(const std::string &url = "", const std::string &token = "") {
        this->apiUrl = url;
        this->authToken = token;
    }

    bool write(SimStep* steps, int stepCount) override;
    void setApiUrl(const std::string &url);
    void setAuthToken(const std::string &token);
};

// ============ DatabaseResultWriter ============

class DatabaseResultWriter : public IResultWriter {
private:
    std::string connectionString;
    std::string tableName;

public:
    DatabaseResultWriter(const std::string &connectionString = "", const std::string &tableName = "simulation_results") {
        this->connectionString = connectionString;
        this->tableName = tableName;
    }

    bool write(SimStep* steps, int stepCount) override;
    void setConnectionString(const std::string &connectionString);
    void setTableName(const std::string &tableName);
};

// ============ Factory Function ============

enum class DestType {
    JSON,
    API,
    DATABASE
};

inline IResultWriter* createResultWriter(DestType type, const char* param = nullptr, const char* param2 = nullptr) {
    switch (type) {
        case DestType::JSON:
            return new JsonResultWriter(param ? param : "", param2 ? param2 : "simulation.json");
        case DestType::API:
            return new ApiResultWriter(param ? param : "", param2 ? param2 : "");
        case DestType::DATABASE:
            return new DatabaseResultWriter(param ? param : "", param2 ? param2 : "simulation_results");
        default:
            return nullptr;
    }
}
