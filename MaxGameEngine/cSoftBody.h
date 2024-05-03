#pragma once

#include <glm/glm.hpp>
#include <glm/vec3.hpp>
#include <vector>

#include "sModelDrawInfo.h"
#include "cMesh.h"

class cSoftBody // Verlet
{
public:
	cSoftBody();
	~cSoftBody();

	struct sNode
	{
		unsigned int nodeIndex = 0;
		
		glm::vec3 currentPosition = glm::vec3(0.0f);
		glm::vec3 oldPosition = glm::vec3(0.0f);

		sVertex* modelVertexPointer = NULL;
	};

	struct sConstraint
	{
		sConstraint() {}

		sNode* nodeA = NULL;
		sNode* nodeB = NULL;

		float restLength = 0.0f;

		unsigned int iterationsCount = 1;

		bool bIsActive = true;
	};

	bool bEnableDebugSphere = false;
	bool bSoftBodyHandledByThread = false;
	bool bSoftBodyInMotion = true;

	float debugSpheresScaleValue = 0.f;
	float dampingFactor = 1.0f;

	std::string debugModelName = "";

	glm::vec3 acceleration = glm::vec3(0.0f);

	glm::vec4 debugSpheresColor = glm::vec4(0.f);

	std::vector < sNode* > nodesList;
	std::vector < std::string > debugSpheresMeshNameList;

	sModelDrawInfo mModelVertexInfo;

	bool CreateSoftBody(sModelDrawInfo& modelDrawInfo, glm::vec3 softBodyPos, glm::mat4 matInitalTransform = glm::mat4(1.0f));

	void UpdateVertexPositions();

	void UpdateNormals(void);

	void VerletUpdate(double deltaTime);

	void ApplyCollision(double deltaTime);

	void SatisfyConstraints(void);

	void CreateRandomBracing(unsigned int numberOfBraces, float minDistanceBetweenVertices);

	void CreateWheelBracing();

	void CreateImaginaryCenterNodeBracing();

	void ApplyDirectionAcceleration(glm::vec3 directedAcceleration);

	void ApplyConstantPosition(glm::vec3 constantPos);

	glm::vec3 GetCurrentPos();

private:
	const double MAX_DELTATIME = 1.0 / 60.0;

	//glm::vec3 softBodyCurrentPos = glm::vec3(0.f);
	//glm::vec3 softBodyOldPos = glm::vec3(0.f);
	glm::vec3 constantPosition = glm::vec3(0.f);

	std::vector < sConstraint* > constraintsList;
	std::vector < sNode* > centerNodesList;

	sNode* mImaginaryCenterNode = NULL;

	void AddImaginaryCenterNode(glm::vec3 softBodyPos);

	void FindCenterNodes();

	void CalculateDampingValue(float deltaTime);

	void CleanZeros(glm::vec3& value);

	float CalculateDistance(sNode* nodeA, sNode* nodeB);
};

