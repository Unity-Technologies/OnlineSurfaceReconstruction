#include "osr/OSRLibrary.h"

#include "osr/Data.h"

struct Mesh
{
	float* Vertices;
	float* Normals;
	int* Triangles;

	int VertexCount;
	int TriangleCount;
};

struct Parameters
{
	float Scale; // > 0, -1 for default
	float Smoothness; // [0, 1), -1 for default
};

using namespace Eigen;
using namespace osr;

class OSR_EXPORT ExtractMeshVisitor : public MeshVisitor
{
	int NextVertex;
	int NextFace;
	
public:
	Mesh Mesh;
	
	
	void begin(unsigned int vertices, unsigned int faces)
	{
		Mesh.VertexCount = vertices;
		Mesh.Vertices = new float[3 * vertices];
		Mesh.Normals = NULL;

		Mesh.TriangleCount = faces;
		Mesh.Triangles = new int32_t[3 * vertices];
	}
	
	void addVertex(const Eigen::Vector3f& position, const Eigen::Vector3f& color)
	{
		Mesh.Vertices[NextVertex + 0] = position.x();
		Mesh.Vertices[NextVertex + 1] = position.y();
		Mesh.Vertices[NextVertex + 2] = position.z();
		NextVertex += 3;
	}
	
	void addFace(unsigned int count, const uint32_t* indices)
	{
		memcpy(Mesh.Triangles + NextFace, indices, sizeof(uint32_t) * count);
		NextFace += count;
	}
	
	void end() { }
};

extern "C" void OSR_EXPORT free_mesh(Mesh* mesh)
{
	delete[] mesh->Triangles;
	delete[] mesh->Normals;
	delete[] mesh->Triangles;
}

extern "C" void OSR_EXPORT process_mesh(Mesh* input, Parameters* parameters, Mesh* output)
{
	MatrixXf V = Eigen::Map<MatrixXf>(input->Vertices, 3, input->VertexCount);
	MatrixXf N = Eigen::Map<MatrixXf>(input->Normals, 3, input->VertexCount);
	MatrixXu F = Eigen::Map<MatrixXu>((unsigned int*)input->Triangles, 3, input->TriangleCount);
	
	Matrix3Xus C; // color, we'll ignore it.
	
	Scan scan(V, N, C, F, "");
	
	Data data;
	data.meshSettings.rosy = std::shared_ptr<IOrientationFieldTraits>(getOrientationFieldTraits(6));
	data.meshSettings.posy = std::shared_ptr<IPositionFieldTraits>(getPositionFieldTraits(6));

	if (parameters->Scale > 0)
		data.meshSettings.setScale(parameters->Scale);
	if (parameters->Smoothness >= 0 && parameters->Smoothness < 1)
		data.meshSettings.smoothness = parameters->Smoothness;
	
	data.AddScan(&scan);
	data.IntegrateScan(&scan);
	
	ExtractMeshVisitor mesh;
	data.extractedMesh.extractFineMesh(mesh, true);
	
	// Note: we don't have normals
	*output = mesh.Mesh;
}