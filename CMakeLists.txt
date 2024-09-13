cmake_minimum_required(VERSION 3.10)
project(animatedNURBS LANGUAGES C)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK REQUIRED gtk+-3.0)
pkg_check_modules(Glib REQUIRED glib-2.0)

include_directories(${GTK_INCLUDE_DIRS} ${Glib_INCLUDE_DIRS})

add_executable(animatedNURBS main.c bezierinterpolator.c)

target_link_libraries(animatedNURBS ${GTK_LIBRARIES} ${Glib_LIBRARIES})

target_compile_options(animatedNURBS PRIVATE ${GTK_CFLAGS_OTHER} ${Glib_CFLAGS_OTHER})
