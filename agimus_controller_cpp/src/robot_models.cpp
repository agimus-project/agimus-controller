#include "agimus_controller/robot_models.hpp"

#include <coal/shape/geometric_shapes.h>

#include <algorithm>
#include <exception>
#include <filesystem>
#include <fstream>
#include <memory>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/multibody/joint/joint-free-flyer.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace agimus_controller {

namespace fs = std::filesystem;

namespace {

/// Checks that at most one of the pair is set and that the file exists.
/// Returns true if the item is set.
bool check_xml_or_path(const std::string& xml, const std::string& path,
                       const std::string& what) {
  if (!xml.empty() && !path.empty()) {
    throw std::invalid_argument(what +
                                ": set either the XML string or the "
                                "file path, not both.");
  }
  if (!path.empty() && !fs::is_regular_file(path)) {
    throw std::runtime_error(what + " must be a valid file path. File: '" +
                             path + "' doesn't exist!");
  }
  return !xml.empty() || !path.empty();
}

std::string read_file(const std::string& path, const std::string& what) {
  std::ifstream file(path);
  if (!file) {
    throw std::runtime_error("Cannot open " + what + " file '" + path + "'.");
  }
  std::stringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

struct LoadedUrdf {
  pinocchio::Model model;
  pinocchio::GeometryModel collision;
  pinocchio::GeometryModel visual;
};

/// Builds pinocchio's model and geometry models from a URDF.
LoadedUrdf load_urdf(const std::string& xml, const std::string& path,
                     bool use_free_flyer, const std::string& meshes_dir,
                     const std::string& what) {
  LoadedUrdf out;
  try {
    const std::string urdf = path.empty() ? xml : read_file(path, what);
    if (use_free_flyer) {
      pinocchio::urdf::buildModelFromXML(urdf, pinocchio::JointModelFreeFlyer(),
                                         out.model);
    } else {
      pinocchio::urdf::buildModelFromXML(urdf, out.model);
    }
    std::vector<std::string> package_dirs;
    if (!meshes_dir.empty()) {
      package_dirs.push_back(fs::absolute(meshes_dir).string());
    }
    const std::pair<pinocchio::GeometryType, pinocchio::GeometryModel*>
        geometries[] = {{pinocchio::COLLISION, &out.collision},
                        {pinocchio::VISUAL, &out.visual}};
    for (const auto& [type, geom] : geometries) {
      std::istringstream stream(urdf);
      pinocchio::urdf::buildGeom(out.model, stream, type, *geom, package_dirs);
    }
  } catch (const std::exception& e) {
    throw std::invalid_argument("Failed to load " + what + " models from " +
                                (path.empty() ? "XML string" : path) + ": " +
                                e.what());
  }
  return out;
}

bool contains(const std::vector<std::string>& names, const std::string& name) {
  return std::find(names.begin(), names.end(), name) != names.end();
}

std::string generate_capsule_name(const std::string& base_name,
                                  const std::vector<std::string>& existing) {
  int i = 0;
  while (contains(existing, base_name + "_capsule_" + std::to_string(i))) ++i;
  return base_name + "_capsule_" + std::to_string(i);
}

/// Replaces each cylinder `<base>_<i>` and the two spheres that close it,
/// `<base>_<i+1>` and `<base>_<i+2>`, by one capsule, and drops the meshes.
void convert_to_capsules(pinocchio::GeometryModel& collision_model,
                         const Eigen::Vector4d& color) {
  std::vector<std::string> capsule_names;
  const auto geom_objects = collision_model.geometryObjects;  // Copy.
  for (const auto& object : geom_objects) {
    const auto& geometry = object.geometry;
    if (auto cylinder =
            std::dynamic_pointer_cast<const coal::Cylinder>(geometry)) {
      const auto sep = object.name.rfind('_');
      const std::string base_name =
          sep == std::string::npos ? "" : object.name.substr(0, sep);
      const auto neighbours = std::count_if(
          geom_objects.begin(), geom_objects.end(),
          [&](const pinocchio::GeometryObject& other) {
            return other.name.find(base_name) != std::string::npos;
          });
      if (neighbours < 3) continue;
      const std::string suffix =
          sep == std::string::npos ? object.name : object.name.substr(sep + 1);
      std::size_t used = 0;
      int id = 0;
      try {
        id = std::stoi(suffix, &used);
      } catch (const std::exception&) {
        used = 0;
      }
      if (used == 0 || used != suffix.size()) {
        throw std::invalid_argument("Cannot convert the cylinder '" +
                                    object.name +
                                    "' to a capsule: its name does not end "
                                    "with an integer.");
      }
      const std::string name = generate_capsule_name(base_name, capsule_names);
      capsule_names.push_back(name);
      pinocchio::GeometryObject capsule(
          name, object.parentJoint, object.parentFrame, object.placement,
          // Bug A17 (ledger), kept for parity until ruled: the second
          // argument is the full length, so this halves the cylinder.
          std::make_shared<coal::Capsule>(cylinder->radius,
                                          cylinder->halfLength));
      capsule.meshColor = color;
      collision_model.addGeometryObject(capsule);
      collision_model.removeGeometryObject(object.name);
      collision_model.removeGeometryObject(base_name + "_" +
                                           std::to_string(id + 1));
      collision_model.removeGeometryObject(base_name + "_" +
                                           std::to_string(id + 2));
    } else if (!std::dynamic_pointer_cast<const coal::Sphere>(geometry) &&
               !std::dynamic_pointer_cast<const coal::Box>(geometry)) {
      collision_model.removeGeometryObject(object.name);  // Useless mesh.
    }
  }
}

}  // namespace

void RobotModelParameters::finalize() {
  if (armature.size() == 0) {
    armature = Eigen::VectorXd::Zero(
        static_cast<Eigen::Index>(moving_joint_names.size()));
  }
  // TODO in Python too: do the same check for a free flyer.
  if (armature.size() != static_cast<Eigen::Index>(moving_joint_names.size()) &&
      !free_flyer) {
    throw std::invalid_argument(
        "armature must have the same size as moving_joint_names. Got " +
        std::to_string(armature.size()) + " and " +
        std::to_string(moving_joint_names.size()) + ".");
  }
  if (!check_xml_or_path(robot_urdf_xml, robot_urdf_path, "Robot URDF")) {
    throw std::invalid_argument("Robot URDF can not be an empty string.");
  }
  check_xml_or_path(env_urdf_xml, env_urdf_path, "Environment URDF");
  check_xml_or_path(srdf_xml, srdf_path, "SRDF");
  if (!urdf_meshes_dir.empty() && !fs::exists(urdf_meshes_dir)) {
    throw std::runtime_error(
        "urdf_meshes_dir must be a valid folder path. Folder: '" +
        urdf_meshes_dir + "' doesn't exist!");
  }
}

RobotModels::RobotModels(RobotModelParameters params)
    : params_(std::move(params)) {
  params_.finalize();

  // Load the full robot, the visual and the collision models.
  auto robot =
      load_urdf(params_.robot_urdf_xml, params_.robot_urdf_path,
                params_.free_flyer, params_.urdf_meshes_dir, "robot URDF");
  full_robot_model_ = std::move(robot.model);
  collision_model_ = std::move(robot.collision);
  visual_model_ = std::move(robot.visual);
  if (!params_.env_urdf_xml.empty() || !params_.env_urdf_path.empty()) {
    const auto env =
        load_urdf(params_.env_urdf_xml, params_.env_urdf_path, false,
                  params_.urdf_meshes_dir, "environment URDF");
    pinocchio::FrameIndex attachment_frame = 0;
    if (!params_.robot_attachment_frame.empty()) {
      // Python takes getFrameId's "not found" value (nframes) without a check.
      if (!env.model.existFrame(params_.robot_attachment_frame)) {
        throw std::invalid_argument(
            "robot_attachment_frame '" + params_.robot_attachment_frame +
            "' is not a frame of the environment URDF.");
      }
      attachment_frame = env.model.getFrameId(params_.robot_attachment_frame);
    }
    // The robot is attached to the environment frame.
    pinocchio::Model appended_model;
    pinocchio::GeometryModel appended_visual;
    pinocchio::appendModel(env.model, full_robot_model_, env.visual,
                           visual_model_, attachment_frame,
                           pinocchio::SE3::Identity(), appended_model,
                           appended_visual);
    visual_model_ = std::move(appended_visual);
    // appendModel adds to its output: the second call needs a fresh model.
    appended_model = pinocchio::Model();
    pinocchio::GeometryModel appended_collision;
    pinocchio::appendModel(env.model, full_robot_model_, env.collision,
                           collision_model_, attachment_frame,
                           pinocchio::SE3::Identity(), appended_model,
                           appended_collision);
    full_robot_model_ = std::move(appended_model);
    collision_model_ = std::move(appended_collision);
  }

  // Lock the joints that do not move.
  const auto& names = full_robot_model_.names;
  for (const auto& name : params_.moving_joint_names) {
    if (!contains(names, name)) {
      std::ostringstream message;
      message << "Moving joint '" << name
              << "' not in the model. Model is:" << full_robot_model_;
      throw std::invalid_argument(message.str());
    }
  }
  std::vector<pinocchio::JointIndex> joints_to_lock;
  for (const auto& name : names) {
    if (name == "universe") continue;
    if (!contains(params_.moving_joint_names, name)) {
      joints_to_lock.push_back(full_robot_model_.getJointId(name));
    }
  }
  Eigen::VectorXd q0 = params_.q0;
  if (q0.size() == 0) {
    q0 = pinocchio::neutral(full_robot_model_);
  } else if (q0.size() != full_robot_model_.nq) {
    throw std::invalid_argument(
        "q0 must have the size of the full robot model (nq = " +
        std::to_string(full_robot_model_.nq) + "). Got " +
        std::to_string(q0.size()) + ".");
  }
  std::vector<pinocchio::GeometryModel> reduced_geometries;
  pinocchio::buildReducedModel(
      full_robot_model_, {collision_model_, visual_model_}, joints_to_lock, q0,
      robot_model_, reduced_geometries);
  collision_model_ = std::move(reduced_geometries[0]);
  visual_model_ = std::move(reduced_geometries[1]);

  if (params_.collision_as_capsule) {
    convert_to_capsules(collision_model_, params_.collision_color);
  }
  if (params_.self_collision) {
    collision_model_.addAllCollisionPairs();
    if (!params_.srdf_path.empty()) {
      pinocchio::srdf::removeCollisionPairs(
          robot_model_, collision_model_,
          fs::absolute(params_.srdf_path).string());
    } else if (!params_.srdf_xml.empty()) {
      pinocchio::srdf::removeCollisionPairsFromXML(
          robot_model_, collision_model_, params_.srdf_xml);
    }
  }
  for (const auto& [name1, name2] : params_.collision_pairs) {
    for (const auto& name : {name1, name2}) {
      if (!collision_model_.existGeometryName(name)) {
        throw std::invalid_argument("Invalid collision pair with name '" +
                                    name + "': not in the collision model.");
      }
    }
    collision_model_.addCollisionPair(
        pinocchio::CollisionPair(collision_model_.getGeometryId(name1),
                                 collision_model_.getGeometryId(name2)));
  }
}

}  // namespace agimus_controller
