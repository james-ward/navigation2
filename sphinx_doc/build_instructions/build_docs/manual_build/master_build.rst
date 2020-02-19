.. _master-build:

Build Navigation2 Master
########################

There are 2 options to build Navigation2 on master branch: using a quickstart setup script or manually.

.. raw:: html

  <button type="button" class="collapsible">Quickstart Using the Initial Setup Script</button>
  <div class="content-collapse">

    <div class="section" id="steps">
    <h3>Steps<a class="headerlink" href="#steps" title="Permalink to this headline">¶</a></h3>
    <p>Install all ROS 2 dependencies from the <a class="reference external" href="https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Development-Setup">ROS2 Installation page</a>.
    Ensure there are no ROS environment variables set in your terminal or <cite>.bashrc</cite> file before taking the steps below.*</p>
    <div class="highlight-bash notranslate"><div class="highlight"><pre><span></span>mkdir &lt;directory_for_workspaces&gt;
    <span class="nb">cd</span> &lt;directory_for_workspaces&gt;
    wget https://raw.githubusercontent.com/ros-planning/navigation2/master/tools/initial_ros_setup.sh
    chmod a+x initial_ros_setup.sh
    ./initial_ros_setup.sh
    </pre></div>
    </div>
    <p><strong>Summary of what’s being done</strong></p>
    <p>The <code class="docutils literal notranslate"><span class="pre">initial_ros_setup.sh</span></code> script downloads three ROS workspaces and then builds them in the correct order. The three workspaces are:</p>
    <ul class="simple">
    <li><p><strong>ROS 2 release</strong>: This is the latest ROS 2 release as defined by the repos file found <a class="reference external" href="https://github.com/ros2/ros2">here</a></p></li>
    <li><p><strong>ROS 2 dependencies</strong>: This is a set of ROS 2 packages that aren’t included in the ROS 2 release yet. However, you need them to be able to build Navigation2. This also includes packages that are part of the ROS 2 release where Navigation2 uses a different version.</p></li>
    <li><p><strong>Navigation2</strong>: This repository.</p></li>
    </ul>
    <p>After all the workspaces are downloaded, run the <cite>navigation2/tools/build_all.sh</cite> script. <cite>build_all.sh</cite> builds each repo in the order listed above using the <cite>colcon build –symlink-install</cite> command.</p>
    </div>
    <div class="section" id="options">
    <h3>Options<a class="headerlink" href="#options" title="Permalink to this headline">¶</a></h3>
    <p>The <cite>initial_ros_setup.sh</cite> accepts the following options:</p>
    <ul class="simple">
    <li><p><cite>–no-ros2</cite> This skips downloading and building the ROS 2 release. Instead it uses the binary packages and <code class="docutils literal notranslate"><span class="pre">setup.sh</span></code> installed in <code class="docutils literal notranslate"><span class="pre">/opt/ros/&lt;ros2-distro&gt;</span></code></p></li>
    <li><p><code class="docutils literal notranslate"><span class="pre">--download-only</span></code> This skips the build steps</p></li>
    </ul>
    </div>

  </div>

.. raw:: html

  <button type="button" class="collapsible">Manually Build Master</button>
  <div class="content-collapse">

    <div class="section" id="build-ros-2-master">
      <h3>Build ROS 2 Master<a class="headerlink" href="#build-ros-2-master" title="Permalink to this headline">¶</a></h3>
      <div class="admonition warning">
      <p class="admonition-title">Warning</p>
      <p>When building ROS 2 from source, make sure that the <cite>ros2.repos</cite> file is from the <cite>master</cite> branch.</p>
      </div>
      <p>Build ROS2 master using the <a class="reference external" href="https://index.ros.org/doc/ros2/Installation">build instructions</a> provided in the ROS2 documentation.</p>
      </div>
      <div class="section" id="build-navigation2-dependencies">
      <h3>Build Navigation2 Dependencies<a class="headerlink" href="#build-navigation2-dependencies" title="Permalink to this headline">¶</a></h3>
      <p>Since we’re not building for a released distribution, we must build the dependencies ourselves rather than using binaries.
      First, source the setup.bash file in the ROS 2 build workspace.</p>
      <blockquote>
      <div><p><code class="docutils literal notranslate"><span class="pre">source</span> <span class="pre">~/ros2_ws/install/setup.bash</span></code></p>
      </div></blockquote>
      <p>Next, we’re going to get the <code class="docutils literal notranslate"><span class="pre">ros2_dependencies.repos</span></code> file from Navigation2.
      Then, use <code class="docutils literal notranslate"><span class="pre">vcs</span></code> to clone the repos and versions in it into a workspace.</p>
      <div class="highlight-bash notranslate"><div class="highlight"><pre><span></span><span class="nb">source</span> ros2_ws/install/setup.bash
      mkdir -p ~/nav2_depend_ws/src
      <span class="nb">cd</span> ~/nav2_depend_ws
      wget https://raw.githubusercontent.com/ros-planning/navigation2/master/tools/ros2_dependencies.repos
      vcs import src &lt; ros2_dependencies.repos
      colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE<span class="o">=</span>Release
      </pre></div>
      </div>
      </div>
      <div class="section" id="id1">
      <h3>Build Navigation2 Master<a class="headerlink" href="#id1" title="Permalink to this headline">¶</a></h3>
      <p>Finally, now that we have ROS2 master and the necessary dependencies, we can now build Navigation2 master itself.
      We’ll source the <code class="docutils literal notranslate"><span class="pre">nav2_depend_ws</span></code>, which will also source the ROS2 master build workspace packages, to build with dependencies.
      The rest of this should look familiar.</p>
      <div class="highlight-bash notranslate"><div class="highlight"><pre><span></span><span class="nb">source</span> ~/nav2_depend_ws/install/setup.bash
      mkdir -p ~/navigation2_ws/src
      <span class="nb">cd</span> ~/navigation2_ws/src
      git clone https://github.com/ros-planning/navigation2.git --branch master
      <span class="nb">cd</span> ~/navigation2_ws
      colcon build --symlink-install
      </pre></div>
      </div>
    </div>

  </div>
