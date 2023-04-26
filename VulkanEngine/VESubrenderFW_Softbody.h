#ifndef VESUBRENDERFWSOFTBODY_H
#define VESUBRENDERFWSOFTBODY_H

namespace ve
{
	/**
		* \brief Subrenderer that manages entities that have one diffuse texture for coloring
		*/
	class VESubrenderFW_Softbody : public VESubrenderFW
	{
	public:
		///Constructor
		VESubrenderFW_Softbody(VERendererForward& renderer)
			: VESubrenderFW(renderer) {};

		///Destructor
		virtual ~VESubrenderFW_Softbody() {};

		///\returns the class of the subrenderer
		virtual veSubrenderClass getClass()
		{
			return VE_SUBRENDERER_CLASS_OBJECT;
		};

		///\returns the type of the subrenderer
		virtual veSubrenderType getType()
		{
			return VE_SUBRENDERER_TYPE_SOFTBODY;
		};

		virtual void initSubrenderer();

		virtual void setDynamicPipelineState(VkCommandBuffer commandBuffer, uint32_t numPass);

		virtual void addEntity(VEEntity* pEntity);
	};
} // namespace ve

#endif
