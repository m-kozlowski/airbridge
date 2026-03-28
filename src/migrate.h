#pragma once

namespace Migrate {
    bool needed();

    void run(void (*lct_fn)(const char *msg));
}
