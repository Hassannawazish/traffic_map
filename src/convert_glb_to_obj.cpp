#include <assimp/Exporter.hpp>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <regex>
#include <sstream>

int main(int argc, char **argv) {
  if (argc != 3) {
    std::cerr << "Usage: convert_glb_to_obj INPUT.glb OUTPUT.obj\n";
    return 1;
  }

  Assimp::Importer importer;
  const aiScene *scene = importer.ReadFile(
      argv[1], aiProcess_Triangulate | aiProcess_JoinIdenticalVertices |
                   aiProcess_GenSmoothNormals | aiProcess_PreTransformVertices);
  if (!scene) {
    std::cerr << "Unable to read GLB: " << importer.GetErrorString() << '\n';
    return 2;
  }

  Assimp::Exporter exporter;
  if (exporter.Export(scene, "obj", argv[2]) != AI_SUCCESS) {
    std::cerr << "Unable to export OBJ: " << exporter.GetErrorString() << '\n';
    return 3;
  }

  const std::filesystem::path output_path(argv[2]);
  const auto directory = output_path.parent_path();
  std::vector<std::string> texture_names(scene->mNumTextures);
  for (unsigned int i = 0; i < scene->mNumTextures; ++i) {
    const aiTexture *texture = scene->mTextures[i];
    if (texture->mHeight != 0) continue;
    std::string extension = texture->achFormatHint;
    if (extension == "jpeg") extension = "jpg";
    if (extension.empty()) extension = "png";
    texture_names[i] = output_path.stem().string() + "_texture_" +
                       std::to_string(i) + "." + extension;
    std::ofstream image(directory / texture_names[i], std::ios::binary);
    image.write(reinterpret_cast<const char *>(texture->pcData), texture->mWidth);
  }

  const auto material_path = directory / (output_path.stem().string() + ".mtl");
  std::ifstream material_input(material_path);
  std::ostringstream material_text;
  std::string line;
  const std::regex embedded_texture(R"(\*([0-9]+))");
  while (std::getline(material_input, line)) {
    std::smatch match;
    if (std::regex_search(line, match, embedded_texture)) {
      const auto index = static_cast<std::size_t>(std::stoul(match[1].str()));
      if (index < texture_names.size() && !texture_names[index].empty())
        line.replace(match.position(), match.length(), texture_names[index]);
    }
    if (line.rfind("newmtl ", 0) == 0)
      std::replace(line.begin() + 7, line.end(), ' ', '_');
    material_text << line << '\n';
  }
  material_input.close();
  std::ofstream(material_path, std::ios::trunc) << material_text.str();

  std::ifstream object_input(output_path);
  std::ostringstream object_text;
  while (std::getline(object_input, line)) {
    if (line.rfind("usemtl ", 0) == 0)
      std::replace(line.begin() + 7, line.end(), ' ', '_');
    object_text << line << '\n';
  }
  object_input.close();
  std::ofstream(output_path, std::ios::trunc) << object_text.str();
  return 0;
}
