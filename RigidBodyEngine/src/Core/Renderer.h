#ifndef __CORE_RENDERER_H__
#define __CORE_RENDERER_H__

#include "../../lib/freeglut/freeglut.h"
#include <string.h>
#include "WindowInfo.h"

class tkWindowInfo;

namespace Core
{
	class tkRenderer {
	public:
		void InitGLUT();
		static void RunGLUT();


	private:
		static void DisplayCallback(void);
		tkWindowInfo* window;
	};
}

#endif /*!__CORE_RENDERER_H__*/
