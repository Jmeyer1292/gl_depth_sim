#include "gl_depth_sim/glad/glad.h"
#include "gl_depth_sim/sim_depth_camera.h"
// OpenGL context
#include <GLFW/glfw3.h>

#include <iostream>

const static std::string vertex_shader_source =
    "#version 330 core\n"
    "layout (location = 0) in vec3 aPos;\n"
    "uniform mat4 mvp;\n"

    "void main()\n"
    "{\n"
      "gl_Position = mvp * vec4(aPos.x, aPos.y, aPos.z, 1.0);\n"
    "}\n";

const static std::string frag_shader_source =
    "#version 330 core\n"
    "out vec4 FragColor;\n"
    "void main()\n"
    "{\n"
      "FragColor = vec4(gl_FragCoord.x / 800.0f, gl_FragCoord.y / 600.0f, 1.0f, 1.0f);\n"
    "}\n";

static Eigen::Matrix4d createProjectionMatrix(const gl_depth_sim::CameraProperties& camera)
{
  Eigen::Matrix4d m (Eigen::Matrix4d::Identity());

  // Organized by column
  m(0,0) = 2.0 * camera.fx / camera.width;
  m(1,1) = 2.0 * camera.fy/ camera.height;
  m(0,2) = 1.0 - 2.0 * camera.cx / camera.width;
  m(1,2) = 2.0 * camera.cy / camera.height - 1.0;
  m(2,2) = (camera.z_far + camera.z_near) / (camera.z_near - camera.z_far);
  m(3,2) = -1.0;
  m(2,3) = 2.0 * camera.z_far * camera.z_near / (camera.z_near- camera.z_far);
  m(3,3) = 0.0;

  return m;
}

gl_depth_sim::SimDepthCamera::SimDepthCamera(const gl_depth_sim::CameraProperties& camera)
  : camera_{camera}
  , proj_{createProjectionMatrix(camera)}
{
  // Load GLFW and OpenGL libraries; create window; create extensions
  initGLFW();

  // Creates an alternate frame buffer for offscreen rendering
  // TODO: Currently un-used
  createGLFramebuffer();

  // Now that opengl is ready, we can load shaders
  depth_program_.reset(new ShaderProgram(vertex_shader_source, frag_shader_source));
}

gl_depth_sim::SimDepthCamera::~SimDepthCamera()
{
  glfwDestroyWindow(window_);
  glDeleteFramebuffers(1, &fbo_);
//  glfwTerminate();
}

static float linearDepth(float depthSample, const float zNear, const float zFar)
{
  if (depthSample == 1.0f) return 0.0;
  depthSample = 2.0 * depthSample - 1.0;
  float zLinear = 2.0 * zNear * zFar / (zFar + zNear - depthSample * (zFar - zNear));
  return zLinear;
}

gl_depth_sim::DepthImage gl_depth_sim::SimDepthCamera::render(const Eigen::Affine3d& pose)
{
  // To OpenGL
  Eigen::Affine3d view_gl = (pose * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())).inverse();

  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glEnable(GL_DEPTH_TEST);
  glClearColor(0.2f, 0.2f, 0.2f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glUseProgram(depth_program_->id());

  // Render each object
  for (const auto& obj : objects_)
  {
    glBindVertexArray(obj.mesh->vao());

    // compute mvp
    Eigen::Projective3d mvp = proj_ * view_gl * obj.pose;
    depth_program_->setUniformMat4("mvp", mvp.matrix().cast<float>());

    glDrawElements(GL_TRIANGLES, obj.mesh->numIndices(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0); // no need to unbind it every time
  }

  // Pull depth data
  DepthImage img;
  img.cols = camera_.width;
  img.rows = camera_.height;
  img.data.resize(img.cols * img.rows);
  glReadPixels(0, 0, camera_.width, camera_.height, GL_DEPTH_COMPONENT, GL_FLOAT, img.data.data());

  // Transform the depth data from clip space 1/w back to linear depth
  for (auto& depth : img.data)
  {
    depth = linearDepth(depth, camera_.z_near, camera_.z_far);
  }

  glfwSwapBuffers(window_);

  return img;
}

bool gl_depth_sim::SimDepthCamera::add(const Mesh& mesh, const Eigen::Affine3d& pose)
{
  std::unique_ptr<RenderableMesh> renderable_mesh (new RenderableMesh(mesh));

  RenderableObjectState state;
  state.mesh = std::move(renderable_mesh);
  state.pose = pose;

  objects_.push_back(std::move(state));

  return true;
}

void gl_depth_sim::SimDepthCamera::initGLFW()
{
//  glfwInit() is called by the glfw_guard object
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  window_ = glfwCreateWindow(camera_.width, camera_.height, "gl_depth_sim", NULL, NULL);
  if (window_ == NULL)
  {
    glfwTerminate();
    throw std::runtime_error("Failed to create GLFW window");
  }

  glfwMakeContextCurrent(window_);

  // glad: load all OpenGL function pointers
  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
  {
    glfwTerminate();
    throw std::runtime_error("Failed to initialize GLAD");
  }

  std::cout << "GL_VERSION: " << GLVersion.major << "." << GLVersion.minor << "\n";

  // Enable clipping [0, 1]
  if (GLAD_GL_ARB_clip_control) { std::cout << "Clip control supported\n"; }
  glClipControl(GL_LOWER_LEFT, GL_ZERO_TO_ONE);

  // Disable V-sync if we can
  glfwSwapInterval(0);
}

void gl_depth_sim::SimDepthCamera::createGLFramebuffer()
{
  // CREATE A FRAME BUFFER OBJECT FOR COLOR & DEPTH
  // Create frame buffer and make it active
  glGenFramebuffers(1, &fbo_);
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_);

  GLuint texColor;
  glGenTextures(1, &texColor);

  GLuint texDepth;
  glGenTextures(1, &texDepth);

  // Create color texture
  glBindTexture(GL_TEXTURE_2D, texColor);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, camera_.width, camera_.height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glBindTexture(GL_TEXTURE_2D, 0);

  // Attach the color to the active fbo
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texColor, 0);

  // Create a depth texture
  glBindTexture(GL_TEXTURE_2D, texDepth);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32F, camera_.width, camera_.height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glBindTexture(GL_TEXTURE_2D, 0);

  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, texDepth, 0);

  if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
    throw std::runtime_error("Framebuffer configuration failed");

  glBindFramebuffer(GL_FRAMEBUFFER, 0);
}
