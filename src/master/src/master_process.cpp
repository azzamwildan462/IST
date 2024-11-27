#include "master/master.hpp"

void Master::process_marker()
{
    // ===========
    // BODY marker
    // ===========
    marker.cube("body_link", "body", 1, get_point(0, 0, 0), get_quat(0, 0, 0), {0.2, 0.2, 0.2, 0.5}, 0.5, 0.5, 1.5);
}