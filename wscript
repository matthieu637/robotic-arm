
import Configure

def detect_X11(conf):
        env = conf.env

        conf.env['LIB_X11'] = ''
        conf.env['X11_FOUND'] = False
        
	conf.env['CPPPATH_X11'] = ['/usr/include/X11', '/usr/local/include', '/usr/include/X11']
        conf.env['LIBPATH_X11'] = ['/usr/lib', '/usr/local/lib']

        res = Configure.find_file('X.h', conf.env['CPPPATH_X11'] )
        conf.check_message('header','X.h', (res != '') , res)
        if (res == '') :
                return 0
        conf.env['X11_FOUND'] = True
        conf.env['LIB_X11'] = ['X11']
        return 1


def set_options(opt) : pass

def configure(conf) :
    return detect_X11(conf)
    

def build(bld):
    model = bld.new_task_gen('cxx', 'staticlib')
    model.source = 'src/ODEFactory.cpp src/ODEObject.cpp src/Prober.cpp src/RoboticArmWorld.cpp src/Utils.cpp'
    model.includes = ['./include/']
    model.target = 'robotic-arm'
    model.uselib = 'ODE BOOST'
    
    model2 = bld.new_task_gen('cxx', 'staticlib')
    model2.source = 'src/ODEFactory.cpp src/ODEObject.cpp src/Prober.cpp src/RoboticArmWorld.cpp src/Utils.cpp \
		    src/Draw.cpp src/RoboticArmWorldView.cpp extern/drawstuff/src/drawstuff.cpp extern/drawstuff/src/x11.cpp'
    model2.includes = ['./include/' , 'extern/drawstuff/include/']
    model2.target = 'robotic-arm-view'
    model2.uselib = 'ODE BOOST GLUT OPENGL X11'


