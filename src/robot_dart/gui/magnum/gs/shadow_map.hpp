#ifndef ROBOT_DART_GUI_MAGNUM_GS_SHADOW_MAP_HPP
#define ROBOT_DART_GUI_MAGNUM_GS_SHADOW_MAP_HPP

#include <robot_dart/gui/magnum/gs/material.hpp>

#include <Corrade/Containers/ArrayView.h>
#include <Corrade/Containers/Reference.h>
#include <Corrade/Utility/Assert.h>

#include <Magnum/GL/AbstractShaderProgram.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Matrix4.h>
#include <Magnum/Shaders/Generic.h>

namespace robot_dart {
    namespace gui {
        namespace magnum {
            namespace gs {
                class ShadowMap : public Magnum::GL::AbstractShaderProgram {
                public:
                    using Position = Magnum::Shaders::Generic3D::Position;
                    using TextureCoordinates = Magnum::Shaders::Generic3D::TextureCoordinates;

                    enum class Flag : Magnum::UnsignedByte {
                        DiffuseTexture = 1 << 0, /**< The shader uses diffuse texture instead of color */
                    };

                    using Flags = Magnum::Containers::EnumSet<Flag>;

                    explicit ShadowMap(Flags flags = {});
                    explicit ShadowMap(Magnum::NoCreateT) noexcept;

                    Flags flags() const;

                    ShadowMap& setTransformationMatrix(const Magnum::Matrix4& matrix);
                    ShadowMap& setProjectionMatrix(const Magnum::Matrix4& matrix);
                    ShadowMap& setMaterial(Material& material);

                private:
                    Flags _flags;
                    Magnum::Int _transformationMatrixUniform{0}, _projectionMatrixUniform{1}, _diffuseColorUniform{2};
                };

                CORRADE_ENUMSET_OPERATORS(ShadowMap::Flags)
            } // namespace gs
        } // namespace magnum
    } // namespace gui
} // namespace robot_dart

#endif