#include "rt_guard.hpp"

#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>

#include <Eigen/Core>
#include <csignal>

namespace agimus_controller::test {

ScopedRtGuard::ScopedRtGuard() noexcept {
#ifdef EIGEN_RUNTIME_NO_MALLOC
  eigen_malloc_was_allowed_ = Eigen::internal::is_malloc_allowed();
  Eigen::internal::set_is_malloc_allowed(false);
#endif
  start_counting();
}

ScopedRtGuard::~ScopedRtGuard() {
  stop_counting();
#ifdef EIGEN_RUNTIME_NO_MALLOC
  Eigen::internal::set_is_malloc_allowed(eigen_malloc_was_allowed_);
#endif
}

bool aborts(void (*fn)(void*), void* context) {
  const pid_t pid = fork();
  if (pid == 0) {
    // Child: default SIGABRT handling (doctest installs its own) and no
    // assertion message on the test output.
    std::signal(SIGABRT, SIG_DFL);
    const int devnull = open("/dev/null", O_WRONLY);
    if (devnull >= 0) {
      dup2(devnull, STDERR_FILENO);
    }
    fn(context);
    _exit(0);
  }
  int status = 0;
  if (pid < 0 || waitpid(pid, &status, 0) != pid) {
    return false;
  }
  return WIFSIGNALED(status) && WTERMSIG(status) == SIGABRT;
}

}  // namespace agimus_controller::test
