#ifndef LINEVIS_SPATIAL_HASHING_DENOISER_HPP
#define LINEVIS_SPATIAL_HASHING_DENOISER_HPP

#include <Graphics/Vulkan/Render/Passes/BlitRenderPass.hpp>
#include "Denoiser.hpp"
#include "EAWDenoiser.hpp"

struct Spatial_Hashing_Uniform_Buffer {
    glm::vec4  cam_pos;
    float      f;        // camera aperture
    float      s_nd;     // normal coarseness
    float      s_p;      // user-defined level of coarseness in pixel
    float      s_min;      // user-defined level of coarseness in pixel
};

struct Spatial_Hashing_Texture_Pack {
    Spatial_Hashing_Uniform_Buffer uniform_buffer;
    sgl::vk::BufferPtr             vk_uniform_buffer;
    sgl::vk::BufferPtr             vk_hash_map_buffer;

    sgl::vk::TexturePtr noisy_texture;
    sgl::vk::TexturePtr normal_texture;
    sgl::vk::TexturePtr position_texture;
    sgl::vk::TexturePtr temp_accum_texture;

    sgl::vk::ImageViewPtr denoised_image;
};


class Spatial_Hashing_Write_Pass;
class Spatial_Hashing_Read_Pass;

class Spatial_Hashing_Denoiser : public Denoiser {
    Spatial_Hashing_Texture_Pack textures;
    sgl::vk::Renderer* renderer;
    sgl::CameraPtr* camera;
    glm::ivec2      resolution;

    std::shared_ptr<Spatial_Hashing_Write_Pass> write_pass;
    std::shared_ptr<Spatial_Hashing_Read_Pass>  read_pass;
    std::shared_ptr<EAWBlitPass>                eaw_pass;

public:
    explicit Spatial_Hashing_Denoiser(sgl::vk::Renderer* renderer, sgl::CameraPtr* camera);

    bool getWantsAccumulatedInput() const override  { return false; }
    bool getWantsGlobalFrameNumber() const override  { return true; }


    DenoiserType getDenoiserType() const override { return DenoiserType::SPATIAL_HASHING; }
    [[nodiscard]] bool getIsEnabled() const override;
    void setOutputImage(sgl::vk::ImageViewPtr& outputImage) override;
    void setFeatureMap(FeatureMapType featureMapType, const sgl::vk::TexturePtr& featureTexture) override;
    [[nodiscard]] bool getUseFeatureMap(FeatureMapType featureMapType) const override;
    void setUseFeatureMap(FeatureMapType featureMapType, bool useFeature) override {};
    void resetFrameNumber() override {};
    void setTemporalDenoisingEnabled(bool enabled) override {} // always temporal denoising, thus unused.
    void denoise() override;
    void recreateSwapchain(uint32_t width, uint32_t height) override;
    bool renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) override;
};

class Spatial_Hashing_Write_Pass : public sgl::vk::ComputePass {
    friend class SVGFDenoiser;
    Spatial_Hashing_Texture_Pack* textures;
    const int computeBlockSize = 16;
    sgl::vk::ComputeDataPtr compute_data;

    void createComputeData(sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) override;
    void _render() override;;

public:
    explicit Spatial_Hashing_Write_Pass(sgl::vk::Renderer* renderer, Spatial_Hashing_Texture_Pack* textures);

protected:
    void loadShader() override;
};

class Spatial_Hashing_Read_Pass : public sgl::vk::ComputePass {
    friend class SVGFDenoiser;
    Spatial_Hashing_Texture_Pack* textures;
    const int computeBlockSize = 16;
    sgl::vk::ComputeDataPtr compute_data;

    void createComputeData(sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) override;
    void _render() override;;

public:
    explicit Spatial_Hashing_Read_Pass(sgl::vk::Renderer* renderer, Spatial_Hashing_Texture_Pack* textures);

protected:
    void loadShader() override;
};

#endif
