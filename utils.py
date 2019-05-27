
import time
import numpy as np

def linear_curve_fitting( x_input, y_input, lambda_ = 0.001):
    '''
    Quick and dirty fitting of a line ax+b =y
    :param x_input: example: np.linspace(0,5,1)
    :param y_input: example: the values to fit
    :param lambda_: regularization
    :return:
    '''


    Phi      = np.ones(shape=( len(x_input),  2))
    Phi[:,0] = x_input

    den = np.linalg.pinv((np.matmul(Phi.T, Phi) + lambda_ * np.eye(Phi.shape[1])))
    w   = np.matmul(np.matmul(den, Phi.T), y_input)

    return w

def error(string_to_plot):

    print("\n\n\n")
    print(string_to_plot)
    print("\n")

    raise ValueError(string_to_plot)


def print_array(xyz, pre='', post='', precision=3):
    '''
    A quick shortcut to print an array of arbitrary elements.
    :param xyz:
    :param pre:
    :param post:
    :return:
    '''


    set_precision_on_display(precision)

    print(pre),
    if type(xyz).__module__ != 'numpy':
        print(np.array(xyz)),
    else:
        print(xyz),
    print(post)
    #
    #
    # print(pre +  str(xyz).strip('[]') + post)

def create_folder_if_does_not_exist(file_path):
    '''
    Create a folder if it does not exist. Note that this code does not recursively create the entire tree
    if subfolders are involved. It can only create the last level folder. Thus, you have to be sure that
    the parent folders exist on the path
    In this example
        ./parent1/parent2/folder3
    only folder3 can be created. If any of the parents does not exist, it returns an error
    :param file_path: ./parent1/parent2/folder3
    :return:
    '''

    if file_path[-1] is not '/':
        file_path = file_path + '/'

    directory = os.path.dirname(file_path)
    try:
        os.stat(directory)
    except:
        os.mkdir(directory)

def set_precision_on_display(precision = 3):
    set_printoptions(precision)

def set_printoptions(precision = 3):
    '''
    When printing in the terminal, set the precision of the numbers
    :param precision:
    :return:
    '''

    # precision = 1
    str_ = '{: 0.%df}' % precision

    np.set_printoptions(formatter={'float': str_ .format})
    #np.set_printoptions(formatter={'float': '{: 0.2f}'.format})


def trapezoid(percentage, minmax, ntraj):

    idx_start = range(0, int(ntraj*percentage))
    idx_end   = range( ntraj-int(ntraj * percentage), ntraj)

    val_start  = np.linspace(minmax[0], minmax[1], len(idx_start))
    val_finish = np.linspace(minmax[1], minmax[0], len(idx_end))
    val_full   = minmax[1]* np.ones(ntraj)

    y = val_full[:]
    y[idx_start] = val_start
    y[idx_end] = val_finish

    if 0:
        utp.quick_figure(y)

    return y


def play_sound_mpg321(file_path, flag_blocking =  True):

    if 0: #seems to be crashing once in a while
        if flag_blocking:
            subprocess.call(['mpg321', '--stereo', file_path])
        else:
            subprocess.Popen(['mpg321',  '--stereo', file_path])
    else:
        os.system(    'mpg321 --stereo, '  +  file_path  )


def make_beep(n_times):
# if 'play' cannot be found, you have to install sox in Linux
# sudo apt install sox

    for k in range(n_times):
        os.system('play  --volume 1 --no-show-progress --null --channels 1 synth %s sine %f' % (0.075, 500))

def pause(pause_time = 0):

    if pause_time == 0:
        raw_input('Paused. Press ENTER to continue.')
    else:
        time.sleep(pause_time)

def save_cpickle(object, filename, protocol = -1):
    """Save an object to a compressed disk file.
       Works well with huge objects.
    """
    file = gzip.GzipFile(filename, 'wb')
    cPickle.dump(object, file, protocol)
    file.close()

def load_cpickle(filename):
    """Loads a compressed object from disk
    """
    file = gzip.GzipFile(filename, 'rb')
    object = cPickle.load(file)
    file.close()

    return object

def resample(n_traj, f_old):
    # resample via linear interpolation

    n_traj_old = f_old.shape[0]
    t_new  = np.linspace(0,1,n_traj)
    t_old  = np.linspace(0, 1, n_traj_old)

    return  interp(t_new, t_old, f_old)


def resample_n_dim(n_traj, f_old):
    '''
    Do a resample on a nDof data.
    Assume rows are time steps
    Assume columns are degrees of freedom
    :param n_traj:
    :param f_old:
    :return:
    '''

    n_dof = f_old.shape[1]

    f_new = np.empty(shape=(n_traj, n_dof))

    for j in range(n_dof):
        f_new[:, j] = resample(n_traj, f_old[:, j])


    return f_new


def interp_nan_1D(y, check_with_plot = False):
    '''
    Given a 1D array where some elements are nan, remove the nan by doing linear interpolation.
    This function is currently only 1D.
    :param  y:
    :return y_out: same as y, but with np.nan replaced with their interpolated values
    '''

    notnan_idx = ~np.isnan(y)
    x_full = np.arange(0, y.shape[0], 1)
    x_in = x_full[notnan_idx]
    y_in = y[notnan_idx]
    y_full = np.interp(x_full, x_in, y_in)

    h = []
    if check_with_plot:
        h = utp.figure_2D_list('remove-nan')
        h['ax'].plot(y, marker='o', color=utp.color('b', 0.0), markersize=15)
        h['ax'].plot(x_full, y_full, marker='o', color=utp.color('r'), markersize=10)

    y_out = y_full

    return y_out, h

def interp(t_new, t_old, f_old):

    if f_old.shape[0] == f_old.size:
        f_new = np.interp(t_new, t_old, f_old)

    else:
        n_dim = f_old.shape[1]
        f_new = np.empty(shape=(t_new.shape[0], n_dim))

        for j in range(n_dim):
            f_new[:, j] = np.interp(t_new, t_old, f_old[:,j])


    return f_new


def ipaddress_retriever(machine_name):
# Retrieve the IP given the computer name.
# Options are:
#    'this_pc'
#    'lemieux4'
#    'maedas_hp_desktop'

    flag_error = 1

    if machine_name == 'this_pc'  :
        return '127.0.0.1'

    if machine_name == 'lemieux4' :
        import socket
        return socket.gethostbyname('lemieux4')

    if machine_name == 'maedas_hp_desktop' :
        import socket
        return socket.gethostbyname('bri-hp750-5')

    if flag_error:
        print('\n\n****No computer found****\n\n')
        error("no computer found when trying to retrieve ip address by host name")


def create_filter_N_joints(n_filter, freq, dt, zeta, K):
# the behavior of the filter depends on the proper combination of freq, dt, zeta
# freq = 10, dt = 0.01 results in very different results if you use freq = 10, dt = 0.005 !!!

    filter = []

    for j in xrange(n_filter):
        filter.append(
            filter_on_line_mass_spring_damper( freq, dt, zeta, K )
        )

    return filter

def filter_N_joints2(x_object, x, x_init_guess, xdot_init_guess):
    # this is an improved version of filter_N_joints. Here you are force to provide
    # xdot_init_guess at all times. Note that
    # xdot_init_guess must be computed outside by differentiation.


    n_dim = len(x_object)

    xfilt = np.zeros(n_dim)
    # xfilt_dot = np.zeros(n_dim)

    for j in xrange(n_dim):

        if 1:#x_init_guess != []:
            #xfilt[j] = x_object[j].next_step(x[j], x_t=x_init_guess[j], xdot_t=xdot_init_guess[j])[0]
            xfilt[j] = x_object[j].next_step(     x[j],   x_t = x_init_guess[j]     )  [0]
        else:
            xfilt[j] = x_object[j].next_step(      x[j]    )[0]

    return xfilt

def filter_N_joints(qLR_MSD_filter, qLR, qLR_init_guess = []):

    nJoints = len(qLR_MSD_filter)

    qLR_filtered = np.zeros(nJoints)

    for j in xrange(nJoints):

        if qLR_init_guess != []:
            qLR_filtered[j] = qLR_MSD_filter[j].next_step(qLR[j], x_t = qLR_init_guess[j])[0]
        else:
            qLR_filtered[j] = qLR_MSD_filter[j].next_step(qLR[j])[0]

    return qLR_filtered

def time_derivative(t, z):
    dz = np.diff(z) / np.diff(t)
    dz = np.hstack((dz, dz[-1]))
    return dz


class struct_fake():
    def __init__(self):
        pass

def flatten_list(l):

    flat = [item for sublist in l for item in sublist]

    return flat


def get_user_answer(message='Are you happy'):

    answer = raw_input(message)
    user_answer = answer in ['true', '1', 't', 'y', 'yes', 'yeah', 'yup', 'certainly', 'uh-huh']

    return user_answer


def string_date_time():

    import datetime
    now = datetime.datetime.now()
    stringy = '%d' % now.year + '%02d' % now.month  + '%02d' % now.day+ '_' + '%02d' % now.hour  + '%02d' % now.minute + '%02d' % now.second

    return stringy


def random_walk(initial_state, n_traj, max_amplitude = []):
    y = initial_state
    result = []
    for _ in range(n_traj):
        result.append(y)
        y += np.random.normal(scale=1)

    y2 = np.array(result)

    if max_amplitude:
        y2zero = y2[0]
        y2 = y2-y2zero

        amp = y2.max()-y2.min()
        y2 = max_amplitude*(y2/amp)

        y2 = y2 + y2zero


    return y2



def r2d(x):
    if type(x) is list:
        x = np.array(x)

    return 360/(2*PI)*x

def d2r(x):
    if type(x) is list:
        x = np.array(x)

    return x*(2*PI)/360

def get_phase_from_trajectory(t, y, eliminate_spike = False):
    # use the convention such that phase starts at zero and goes counterclockwise as the angle
    # increases. The same as used in the paper

    y = y - y[0]

    # first make sure trajectory is increasing
    if y[0] > y[-1]:
        y = -y

    # eliminate the negative spike that shows in cbi during real experiments
    if eliminate_spike:
        v, idx = np.min(y), np.argmin(y)
        y = y - v
        y[0:idx] = 0

    yz = y[:]

    # normalized data
    y_n = y/amplitude(y)

    # effectively compute the phase
    y = -y
    ydot = np.diff(y) / np.diff(t)
    ydot = np.hstack((ydot, ydot[-1]))

    phase = -np.arctan2( ydot, y + 0.5 *amplitude(y)  )
    phase = np.unwrap(phase)

    return phase, yz, y_n



def get_dropbox_path():

    try:
        path_  =os.environ['DROPBOXX']
        if path_[-1] == 'x':
            path_ += '/'

        return path_
    except:
        print('No dropbox environment variable found')
        return 'not found'



def smooth_simple(a,WSZ):
    # a: NumPy 1-D array containing the data to be smoothed
    # WSZ: smoothing window size needs, which must be odd number,
    # as in the original MATLAB implementation
    out0 = np.convolve(a,np.ones(WSZ,dtype=int),'valid')/WSZ
    r = np.arange(1,WSZ-1,2)
    start = np.cumsum(a[:WSZ-1])[::2]/r
    stop = (np.cumsum(a[:-WSZ:-1])[::2]/r)[::-1]
    return np.concatenate((  start , out0, stop  ))


# computes the centroid, regardless of the dimension of the input
# X columns are time steps, rows span the dimension.
def centroid(x):
    return x.sum(axis=1)/x.shape[1]


# for orthogonal projection when using 3D plots
def orthogonal_proj(zfront, zback):
    a = (zfront + zback) / (zfront - zback)
    b = -2 * (zfront * zback) / (zfront - zback)

    print('orthogonal being used here.')

    return np.array([[1, 0, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, a, b],
                     [0, 0, -0.0001, zback]])




class MyTic():

    def __init__(self):
        self.tic = time.time()

    def toc(self, verbose=0):
        elapsed_time = time.time() - self.tic
        if verbose:
            print('Elapsed time:' , elapsed_time, '(sec)')
        return elapsed_time




def dbstop_if_error(type, value, tb):
    if hasattr(sys, 'ps1') or not sys.stderr.isatty():
    # we are in interactive mode or we don't have a tty-like
    # device, so we call the default hook
        sys.__excepthook__(type, value, tb)
    else:
        import traceback, pdb
        # we are NOT in interactive mode, print the exception...
        traceback.print_exception(type, value, tb)
        print
        # ...then start the debugger in post-mortem mode.
        # pdb.pm() # deprecated
        pdb.post_mortem(tb) # more "modern"

# sys.excepthook = dbstop_if_error

def stop():
    sys.exit()

def odometry(y):
    return np.cumsum(np.abs(np.diff(y)))

def probability_of_traj_Pi2(C):
    n_reps = len(C)

    # gjm: The theoretical expS is replaced by this heuristic here. See Pi2
    #      paper ICRA 2010.
    #
    # compute the exponentiated cost with the special trick to automatically
    # adjust the lambda scaling parameter
    if type(C) is list:
        C = np.array(C)

    maxS = C.max()
    minS = C.min()
    h = 10  # this is the scaling parameters in side of the exp() function (see README.pdf)

    den = (maxS - minS) * np.ones(shape=(1, n_reps))
    num = C - minS * np.ones(shape=(1, n_reps))
    expS = np.exp(-h * num / den)

    return expS / (expS.sum() * np.ones(shape=(1, n_reps)))

def count_every_N(ctr, N):
    '''
    :param ctr: incremental counter
    :param N:   how many skips
    :return:    1 or 0
    '''

    if ctr % N == 0:
        return 1
    else:
        return 0

def amplitude(y):
    return np.abs(np.max(y) - np.min(y))

# =============================================
# Deprecated filter. Only maintained here to keep legacy code working
# =============================================
class filter_on_line_mass_spring_damper():
    # this is a simple 2nd order low pass filter based on the dynamics of mass spring damper.
    # the force that actuates the mess is the noisy measurement. And the position of the mass
    # is the filtered signal.

    def __init__(self, freq=3, dt=0.01, zeta=0.74, K=1):

        self.freq = freq
        self.omega_n = 2 * np.pi * freq
        self.zeta = zeta
        self.K = K
        self.dt = dt
        self.x_t    = 0
        self.xdot_t = 0

    def next_step(self, u, x_t=[], xdot_t=[]):
        # this dt is not related to the dt of the real loop
        # as such, the self.xdot is also not related to the real-time loop

        if x_t == []:
            x_t = self.x_t
        if xdot_t == []:
            xdot_t = self.xdot_t



        # integrate mass spring damper
        y_t = xdot_t

        y_t1 = y_t + (self.K * u - x_t - 2 * self.zeta / self.omega_n * y_t) * self.dt * self.omega_n ** 2
        x_t1 = x_t + self.dt * y_t1
        #x_t1 = x_t + self.dt * y_t

        # TODO: delete this
        if 0:
            x_t1 = 0
            y_t1 = 0

        self.x_t = x_t1
        self.xdot_t = y_t1

        return x_t1, y_t1


    def run_filter_offline(self, y_raw):


        self.x_t
        self.xdot_t

        y_filt = np.empty(y_raw.shape)
        n_traj = len(y_raw)

        for t in range(n_traj):

            if self.x_y == []:
                y_filt[t] = self.next_step( y_raw[t] )[0]
            else:
                y_filt[t] = self.next_step( y_raw[t], y_filt[t-1]  )[0]


        return y_filt


def keyboard_shortcut_pycharm():
    print('\n*********************************')
    print('Current pycharm keyboard shortcuts')
    print('F4:  stop')
    print('Shift + F5: run')
    print('F5:  resume when in debugging mode until next breakpoint or end of file')
    print('F6:  resume when in debugging mode until cursor')
    print('F7:  toggle breakpoints')
    print('F8:  start debugging')
    print('F9:  evaluate current line')
    print('F10: evaluate to next line')
    print('F11: smart step into')
    print('F12: full screen')
    print('*********************************\n')

    print('ctr + shift + number: bookmark line')
    print('ctr + number: go to bookmark')
    print('*********************************\n')



















