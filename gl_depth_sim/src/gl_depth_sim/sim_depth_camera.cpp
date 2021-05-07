#include "gl_depth_sim/sim_depth_camera.h"
#include "gl_depth_sim/glad/egl.h"
#include "gl_depth_sim/glad/gles2.h"

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

/**
 * @brief Creates a projection matrix using the REVERSE-Z method! Maps far values to 0.0 to
 * make for better accuracy at distance.
 */
static Eigen::Matrix4d createProjectionMatrix(const gl_depth_sim::CameraProperties& camera)
{
  Eigen::Matrix4d m (Eigen::Matrix4d::Identity());
  // Organized by column
  m(0,0) = 2.0 * camera.fx / camera.width;
  m(1,1) = 2.0 * camera.fy/ camera.height;
  m(0,2) = 1.0 - 2.0 * camera.cx / camera.width;
  m(1,2) = 2.0 * camera.cy / camera.height - 1.0;
  m(2,2) = camera.z_near / (camera.z_far - camera.z_near);
  m(3,2) = -1.0;
  m(2,3) = camera.z_far * camera.z_near / (camera.z_far - camera.z_near);
  m(3,3) = 0.0;

  return m;
}

gl_depth_sim::SimDepthCamera::SimDepthCamera(const gl_depth_sim::CameraProperties& camera)
  : camera_(camera)
  , proj_(createProjectionMatrix(camera))
{
  // Load GLFW and OpenGL libraries; create window; create extensions
  initGLFW();

  // Creates an alternate frame buffer for offscreen rendering
  createGLFramebuffer();

  // Now that opengl is ready, we can load shaders
  depth_program_.reset(new ShaderProgram{vertex_shader_source, frag_shader_source});
}

gl_depth_sim::SimDepthCamera::~SimDepthCamera()
{
  glDeleteFramebuffers(1, &fbo_);
}


gl_depth_sim::DepthImage gl_depth_sim::SimDepthCamera::render(const Eigen::Isometry3d& pose)
{
  // To OpenGL
  Eigen::Isometry3d view_gl = (pose * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())).inverse();

  glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_GREATER);
  glClearDepthf(0.0f);
  glClear(GL_DEPTH_BUFFER_BIT);

  glUseProgram(depth_program_->id());

  // Render each object
  for (const auto& obj : objects_)
  {
    glBindVertexArray(obj.second.mesh->vao());

    // compute mvp
    Eigen::Projective3d mvp = proj_ * view_gl * obj.second.pose;
    depth_program_->setUniformMat4("mvp", mvp.matrix().cast<float>());

    glDrawElements(GL_TRIANGLES, obj.second.mesh->numIndices(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0); // no need to unbind it every time
  }

  // Pull depth data back from the GPU
  DepthImage img (camera_.height, camera_.width); // (rows, cols)
  glReadPixels(0, 0, camera_.width, camera_.height, GL_DEPTH_COMPONENT, GL_FLOAT, img.data.data());

  // Transform the depth data from clip space 1/w back to linear depth. This is a subsection of the inverse of
  // the projection matrix.
  // eye_depth(b) = (zf * zn) / (b * (zf - zn) + zn)
  // where zn = near z clipping distance, zf = far z clipping distance, b = sample from depth buffer
  // in the case of b == 0.0f, we return 0.0f linear distance
  const float zf_zn = camera_.z_far * camera_.z_near;
  const float zf_minus_zn = camera_.z_far - camera_.z_near;
  for (auto& depth : img.data)
  {
    if (depth != 0.0f)
    {
      depth = zf_zn / (depth * (zf_minus_zn) + camera_.z_near);
    }
  }

  /*
  glfwSwapBuffers(window_);
  */
  eglSwapBuffers(display_, EGL_NO_SURFACE);

  return img;
}

bool gl_depth_sim::SimDepthCamera::add(const std::string mesh_id, const Mesh& mesh, const Eigen::Isometry3d& pose)
{
  std::unique_ptr<RenderableMesh> renderable_mesh (new RenderableMesh{mesh});

  RenderableObjectState state;
  state.mesh = std::move(renderable_mesh);
  state.pose = pose;

  objects_[mesh_id] = std::move(state);
  return true;
}

bool gl_depth_sim::SimDepthCamera::add( const Mesh& mesh, const Eigen::Isometry3d& pose)
{
  const std::string mesh_id = "mesh" + std::to_string(rand()%1000);
  std::unique_ptr<RenderableMesh> renderable_mesh (new RenderableMesh{mesh});

  RenderableObjectState state;
  state.mesh = std::move(renderable_mesh);
  state.pose = pose;

  objects_[mesh_id] = std::move(state);
  return true;
}

bool gl_depth_sim::SimDepthCamera::move(const std::string mesh_id, const Eigen::Isometry3d &pose)
{
  objects_[mesh_id].pose = pose;
  return true;
}



void gl_depth_sim::SimDepthCamera::initGLFW()
{
  /*
  //  glfwInit() is called by the glfw_guard object
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_VISIBLE, false);

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
  */


    int egl_version = gladLoaderLoadEGL(NULL);
    if (!egl_version) {
        throw std::runtime_error("Unable to load EGL.");
    }

    std::cout << "Initial EGL_VERSION: " << GLAD_VERSION_MAJOR(egl_version) << "." << GLAD_VERSION_MINOR(egl_version) << "\n";

    display_ = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    if (display_ == EGL_NO_DISPLAY) {
        throw std::runtime_error("Got no EGL display");
    }

    if (!eglInitialize(display_, NULL, NULL)) {
        throw std::runtime_error("Unable to initialize EGL");
    }

    egl_version = gladLoaderLoadEGL(display_);
    if (!egl_version) {
        throw std::runtime_error("Unable to reload EGL");
    }

    std::cout << "Reloaded EGL_VERSION: " << GLAD_VERSION_MAJOR(egl_version) << "." << GLAD_VERSION_MINOR(egl_version) << "\n";

    eglBindAPI(EGL_OPENGL_API);

    EGLint attr[] = {
      EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
      EGL_BLUE_SIZE, 8,
      EGL_RED_SIZE, 8,
      EGL_GREEN_SIZE, 8,
      EGL_DEPTH_SIZE, 24,
      EGL_COLOR_BUFFER_TYPE, EGL_RGB_BUFFER,
      EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT,
      EGL_CONFORMANT, EGL_OPENGL_BIT,
      EGL_NONE
    };
    EGLConfig egl_config;
    EGLint num_config;
    if (!eglChooseConfig(display_, attr, &egl_config, 1, &num_config)) {
      throw std::runtime_error("Failed to choose config (eglError: " + std::to_string(eglGetError()) + ")");
   }

    EGLint ctxattr[] = {
      EGL_CONTEXT_MAJOR_VERSION, 4,
      EGL_CONTEXT_MINOR_VERSION, 1,
      EGL_CONTEXT_OPENGL_PROFILE_MASK,
      EGL_CONTEXT_OPENGL_CORE_PROFILE_BIT,
      EGL_NONE
    };

    EGLContext egl_context = eglCreateContext(display_, egl_config, EGL_NO_CONTEXT, ctxattr);
    if (egl_context == EGL_NO_CONTEXT) {
      throw std::runtime_error("Unable to create EGL context (eglError: " + std::to_string(eglGetError()) + ")");
    }

    eglMakeCurrent(display_, EGL_NO_SURFACE, EGL_NO_SURFACE, egl_context);

    int gles_version = gladLoaderLoadGLES2();
    if(!gles_version) {
      throw std::runtime_error("Unable to load GLES");
    }

    std::cout << "GLES_VERSION: " << GLAD_VERSION_MAJOR(gles_version) << "." << GLAD_VERSION_MINOR(gles_version) << "\n";

  /*
  // Enable clipping [0, 1]
  if (GLAD_GL_ARB_clip_control)
  {
    std::cout << "Clip control supported\n";
    glClipControl(GL_LOWER_LEFT, GL_ZERO_TO_ONE);
  }
  else
  {
    throw std::runtime_error("Your OpenGL context does not support glClipControl");
  }
  */

  // Disable V-sync if we can
  // eglSwapInterval(egl_display, 0);
}

void gl_depth_sim::SimDepthCamera::createGLFramebuffer()
{
  // Create frame buffer and make it active
  glGenFramebuffers(1, &fbo_);
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_);

  // Create a depth texture
  GLuint texDepth;
  glGenTextures(1, &texDepth);
  // Configure the texture
  glBindTexture(GL_TEXTURE_2D, texDepth);
  // Note the use of 32bit floating point depth buffer
  glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32F, camera_.width, camera_.height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glBindTexture(GL_TEXTURE_2D, 0);
  // Bind the depth texture to our frame buffer
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, texDepth, 0);

  if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
    throw std::runtime_error("Framebuffer configuration failed");

  glBindFramebuffer(GL_FRAMEBUFFER, 0);
}
