// SPDX-FileCopyrightText: Copyright 2024 Kenji Koide
// SPDX-License-Identifier: MIT
#pragma once

#ifdef SMALL_GICP_STD_PTR
#include <memory>
#else
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#endif

namespace small_gicp {

#ifdef SMALL_GICP_STD_PTR
template <typename T>
using shared_ptr = std::shared_ptr<T>;

template <typename T, typename... Args>
shared_ptr<T> make_shared(Args&&... args) {
  return std::make_shared<T>(std::forward<Args>(args)...);
}
#else
template <typename T>
using shared_ptr = boost::shared_ptr<T>;

template <typename T, typename... Args>
shared_ptr<T> make_shared(Args&&... args) {
  return boost::make_shared<T>(std::forward<Args>(args)...);
}
#endif

}  // namespace small_gicp
