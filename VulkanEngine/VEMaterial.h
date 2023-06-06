/**
* The Vienna Vulkan Engine
*
* (c) bei Helmut Hlavacs, University of Vienna
*
*/

#ifndef VEMATERIAL_H
#define VEMATERIAL_H

namespace ve
{
	/**
		*
		* \brief Store texture data
		*
		* Store an image and a sampler as shader resource.
		*/

	struct VETexture : VENamedClass
	{
		VkImage m_image = VK_NULL_HANDLE; ///<image handle

		///\brief Vulkan handles for the image
		VkDescriptorImageInfo m_imageInfo = {
			VK_NULL_HANDLE, ///<sampler
			VK_NULL_HANDLE, ///<image view
			VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL ///<layout
		};
		VmaAllocation m_deviceAllocation = nullptr; ///<VMA allocation info
		VkExtent2D m_extent = { 0, 0 }; ///<map extent
		VkFormat m_format; ///<texture format

		//VETexture(std::string name, gli::texture_cube &texCube, VkImageCreateFlags flags = VK_IMAGE_CREATE_CUBE_COMPATIBLE_BIT, VkImageViewType viewType = VK_IMAGE_VIEW_TYPE_CUBE);
		VETexture(std::string name, std::string &basedir, std::vector<std::string> texNames, VkImageCreateFlags flags = 0, VkImageViewType viewtype = VK_IMAGE_VIEW_TYPE_2D);

		///Empty constructor
		VETexture(std::string name)
			: VENamedClass(name) {};

		~VETexture();
	};

	/**
		*
		* \brief Store material data
		*
		* VEMaterial stores material data, including several textures, and color data.
		*/

	class VEMaterial : public VENamedClass
	{
	public:
		aiShadingMode shading = aiShadingMode_Phong; ///<Default shading model

		VETexture *mapDiffuse = nullptr; ///<Diiffuse texture
		VETexture *mapBump = nullptr; ///<Bump map
		VETexture *mapNormal = nullptr; ///<Normal map
		VETexture *mapHeight = nullptr; ///<Height map
		glm::vec4 color = glm::vec4(0.5f, 0.5f, 0.5f, 1.0f); ///<General color of the entity

		///Constructor
		VEMaterial(std::string name)
			: VENamedClass(name),
			mapDiffuse(nullptr),
			mapBump(nullptr),
			mapNormal(nullptr),
			mapHeight(nullptr),
			color(glm::vec4(0.0f, 0.0f, 0.0f, 1.0f)) {};

		~VEMaterial();
	};

	/**
		*
		* \brief Store a mesh in a Vulkan vertex and index buffer
		*
		* VEMesh stores a mesh in a Vulkan vertex and index buffer. For both buffers, also the VMA
		* allocation information is stored.
		*
		*/

	class VEMesh : public VENamedClass
	{
	protected:
		VEMesh(std::string name);																	// MOD!
																									// VVE Base Code Modification. Reason: VEClothMesh needs to create buffers in its own way
	public:
		uint32_t m_vertexCount = 0; ///<Number of vertices in the vertex buffer
		uint32_t m_indexCount = 0; ///<Number of indices in the index buffer
		VkBuffer m_vertexBuffer = VK_NULL_HANDLE; ///<Vulkan vertex buffer handle
		VmaAllocation m_vertexBufferAllocation = nullptr; ///<VMA allocation info
		VkBuffer m_indexBuffer = VK_NULL_HANDLE; ///<Vulkan index buffer handle
		VmaAllocation m_indexBufferAllocation = nullptr; ///<VMA allocation info
		glm::vec3 m_boundingSphereCenter = glm::vec3(0.0f, 0.0f, 0.0f); ///<center of bounding sphere in local space
		float m_boundingSphereRadius = 1.0; ///<Radius of bounding sphere in local space

		VEMesh(std::string name, const aiMesh *paiMesh);

		VEMesh(std::string name, std::vector<vh::vhVertex> &vertices, std::vector<uint32_t> &indices);

		virtual ~VEMesh();																			// MOD!
																									// VVE Base Code Modification: Reason: ~VEClothMesh needs to be called also when working with VEMesh Pointers
	};


	//----------------------------------Cloth-Simulation-Stuff--------------------------------------
	// by Felix Neumann

	class VEClothMesh : public VEMesh {
	private:
		VkBuffer m_stagingBuffer = VK_NULL_HANDLE;
		VmaAllocation m_stagingBufferAllocation = nullptr;
		VkDeviceSize m_bufferSize = 0;
		std::vector<vh::vhVertex> m_vertices;
		std::vector<uint32_t> m_indices;
		void* m_ptrToStageBufMem = nullptr;
	public:
		VEClothMesh(const aiMesh* paiMesh);

		virtual ~VEClothMesh();

		void updateVertices(std::vector<vh::vhVertex>& vertices);

		const std::vector<vh::vhVertex>& getVertices() const;
		const std::vector<uint32_t>& getIndices() const;
	private:
		void loadFromAiMesh(const aiMesh* paiMesh);
		void updateBoundingSphere();
		void updateNormals(std::vector<vh::vhVertex>& vertices);
		static std::vector<vh::vhVertex> createVerticesWithBack(
			std::vector<vh::vhVertex>& vertices);
		static std::vector<uint32_t> createIndicesWithBack(std::vector<uint32_t>& indices,
			uint32_t highestIndex);
		void createBuffers();
	};

} // namespace ve

#endif
