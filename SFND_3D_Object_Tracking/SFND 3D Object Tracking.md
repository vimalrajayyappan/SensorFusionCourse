# SFND 3D Object Tracking:

With the knowledge from previous project on Keypoint Detectors, Descirptors, Matching and knowing how Lidar detection works, we levelled up further
in this project by utilising those informations to perform 3D tracking on objects. This invloves `(1)detection of objects in camera frame using YOLO- Deep learning framework`, `(2)associating the objects between each frames based on keypoint matches and bounding box we have`, `(3)Compute Median Distance ratio on the Keypoint matches to find camera based
TTC(Time TO COllision)`, and `(4)Clustering Lidar Points using bounding box between image frames to compute Lidar Only TTC`. Overall a great learning and yes its a lengthy course :D.

Following are the rubric points 

## FP.1 Match 3D Objects:
`matchBoundingBoxes()` method in `camFusion_Student.cpp` is implemented to associate bounding boxes based on keypoint matches between the frames. The logic involves choosing
only corresponding ROI(region of interest) holds the filtered and matched keypoints. Then we iterate the bounding boxes for both previous and current frame, to store the
number of matched keypoints captured in each combination of bounding boxes. The combination with max-counts of keypoints are grouped and returned as associated bounding box
in the form of map.

## FP.2 Compute Lidar-based TTC

The clustering logic for Lidar points is already implemeneted which involves transforming the lidar points to camera frame and group them based on ROI(bounding box) in camera frame. Also there is a shrink factor, which shrinks the grouped lidar points so that the outliers along th eedges which are prone to errors are neglected.

Then the `computeTTCLidar()` function is implemented in `camFusion_Student.cpp`, where I just iterated through Lidar points in both the frames and found the corresponding mean
value both along X and Y. Computed the relative distance, which is then used to compute TTC. Using only closest point, will pick some outliers impacting TTC result.

## FP.3 Associate Keypoint Correspondences with Bounding Boxes
`clusterKptMatchesWithROI()` method in `camFusion_Student.cpp` is implemented. The logic follows, iterating all the keypoints in the bounding box and compute the euclidean distance with their corresponding matches in previous frame. Those keypoints are choosen, whose distance is approximately 1.2x closer to mean distance.These keypoints are then finalised for median distance ratio calculation for computing TTC.

## FP.4 Compute Camera-based TTC
`computeTTCCamera()` in `camFusion_Student.cpp` is implemented. Here the selected keypoints from FP.3 are taken and distance between one keypoint to everyother keypoints are computed for both current(disCurr) and previous frames(disPrev). Then the ratio (disCurr/disPrev) is calculated and finally their median is derived. This is called _Median Distance Ratio_ which is finally used  to calculate camera based TTC.


## FP.5 Performance Evaluation 1:

I have attached the TTC calculated by Lidar below. Even though Lidar is a very good sensor, the TTC calculation highly depends on choosing one best point in the point cloud.
I have seen deviations in the marked cells below. Though they are not too high, but its still need to be taken care. The main reason I found is accompaniying additional filter
algorithms to choose points that are effective would help reducing this error. Also the mean lidar point which we had computed is no guarntee cant be an outlier.

## FP.6 Performance Evaluation 2
Since we have no ground truth data to verify, taking LiDAR TTC as an initial approximation to validate against the values generated for image based TTCs.

I have run all the possible detector-descriptor combinations and the results are attached in the spreadsheet. As you can see Detector types _FAST, BRISK and SIFT_
there are TTC values which really not in the true range. Sometimes the TTCs are in the ranges of (40, 70 and even 90s). Even though _SIFT_ is comparitively better among the
mentioned 3 detectors, still it shows the values in 20s which is not recommended. 

Comparing the rest, _ORB and HARRIS_ detectors, where the results are more erratic, as you can see the values in range of 500s for 10th image in _[ORB-SIFT]_ combination and
13th image of _[HARRIS+SIFT]_ combination.Some cases shows inf and negative values which are clearly due to poor selection of keypoints. Which is again not recommended.

Finally comparing _SHITOMASI and AKAZE_ - Gauging the values , I found SHITOMASI perfomed better as the change in values with respect to corresponding next frames are not 
that high and also considerably in the range compared to LiDAR and also on intutive understadning. Though the AKAZE perform secondly some values in the range of 17s are still
questionable. So I would choose *_[SHITOMASI + BRISK] or [SHITOMASI + ORB]_* combinations for TTC.











# Dillinger
## _The Last Markdown Editor, Ever_

[![N|Solid](https://cldup.com/dTxpPi9lDf.thumb.png)](https://nodesource.com/products/nsolid)

[![Build Status](https://travis-ci.org/joemccann/dillinger.svg?branch=master)](https://travis-ci.org/joemccann/dillinger)

Dillinger is a cloud-enabled, mobile-ready, offline-storage compatible,
AngularJS-powered HTML5 Markdown editor.

- Type some Markdown on the left
- See HTML in the right
- ✨Magic ✨

## Features

- Import a HTML file and watch it magically convert to Markdown
- Drag and drop images (requires your Dropbox account be linked)
- Import and save files from GitHub, Dropbox, Google Drive and One Drive
- Drag and drop markdown and HTML files into Dillinger
- Export documents as Markdown, HTML and PDF

Markdown is a lightweight markup language based on the formatting conventions
that people naturally use in email.
As [John Gruber] writes on the [Markdown site][df1]

> The overriding design goal for Markdown's
> formatting syntax is to make it as readable
> as possible. The idea is that a
> Markdown-formatted document should be
> publishable as-is, as plain text, without
> looking like it's been marked up with tags
> or formatting instructions.

This text you see here is *actually- written in Markdown! To get a feel
for Markdown's syntax, type some text into the left window and
watch the results in the right.

## Tech

Dillinger uses a number of open source projects to work properly:

- [AngularJS] - HTML enhanced for web apps!
- [Ace Editor] - awesome web-based text editor
- [markdown-it] - Markdown parser done right. Fast and easy to extend.
- [Twitter Bootstrap] - great UI boilerplate for modern web apps
- [node.js] - evented I/O for the backend
- [Express] - fast node.js network app framework [@tjholowaychuk]
- [Gulp] - the streaming build system
- [Breakdance](https://breakdance.github.io/breakdance/) - HTML
to Markdown converter
- [jQuery] - duh

And of course Dillinger itself is open source with a [public repository][dill]
 on GitHub.

## Installation

Dillinger requires [Node.js](https://nodejs.org/) v10+ to run.

Install the dependencies and devDependencies and start the server.

```sh
cd dillinger
npm i
node app
```

For production environments...

```sh
npm install --production
NODE_ENV=production node app
```

## Plugins

Dillinger is currently extended with the following plugins.
Instructions on how to use them in your own application are linked below.

| Plugin | README |
| ------ | ------ |
| Dropbox | [plugins/dropbox/README.md][PlDb] |
| GitHub | [plugins/github/README.md][PlGh] |
| Google Drive | [plugins/googledrive/README.md][PlGd] |
| OneDrive | [plugins/onedrive/README.md][PlOd] |
| Medium | [plugins/medium/README.md][PlMe] |
| Google Analytics | [plugins/googleanalytics/README.md][PlGa] |

## Development

Want to contribute? Great!

Dillinger uses Gulp + Webpack for fast developing.
Make a change in your file and instantaneously see your updates!

Open your favorite Terminal and run these commands.

First Tab:

```sh
node app
```

Second Tab:

```sh
gulp watch
```

(optional) Third:

```sh
karma test
```

#### Building for source

For production release:

```sh
gulp build --prod
```

Generating pre-built zip archives for distribution:

```sh
gulp build dist --prod
```

## Docker

Dillinger is very easy to install and deploy in a Docker container.

By default, the Docker will expose port 8080, so change this within the
Dockerfile if necessary. When ready, simply use the Dockerfile to
build the image.

```sh
cd dillinger
docker build -t <youruser>/dillinger:${package.json.version} .
```

This will create the dillinger image and pull in the necessary dependencies.
Be sure to swap out `${package.json.version}` with the actual
version of Dillinger.

Once done, run the Docker image and map the port to whatever you wish on
your host. In this example, we simply map port 8000 of the host to
port 8080 of the Docker (or whatever port was exposed in the Dockerfile):

```sh
docker run -d -p 8000:8080 --restart=always --cap-add=SYS_ADMIN --name=dillinger <youruser>/dillinger:${package.json.version}
```

> Note: `--capt-add=SYS-ADMIN` is required for PDF rendering.

Verify the deployment by navigating to your server address in
your preferred browser.

```sh
127.0.0.1:8000
```

## License

MIT

**Free Software, Hell Yeah!**

[//]: # (These are reference links used in the body of this note and get stripped out when the markdown processor does its job. There is no need to format nicely because it shouldn't be seen. Thanks SO - http://stackoverflow.com/questions/4823468/store-comments-in-markdown-syntax)

   [dill]: <https://github.com/joemccann/dillinger>
   [git-repo-url]: <https://github.com/joemccann/dillinger.git>
   [john gruber]: <http://daringfireball.net>
   [df1]: <http://daringfireball.net/projects/markdown/>
   [markdown-it]: <https://github.com/markdown-it/markdown-it>
   [Ace Editor]: <http://ace.ajax.org>
   [node.js]: <http://nodejs.org>
   [Twitter Bootstrap]: <http://twitter.github.com/bootstrap/>
   [jQuery]: <http://jquery.com>
   [@tjholowaychuk]: <http://twitter.com/tjholowaychuk>
   [express]: <http://expressjs.com>
   [AngularJS]: <http://angularjs.org>
   [Gulp]: <http://gulpjs.com>

   [PlDb]: <https://github.com/joemccann/dillinger/tree/master/plugins/dropbox/README.md>
   [PlGh]: <https://github.com/joemccann/dillinger/tree/master/plugins/github/README.md>
   [PlGd]: <https://github.com/joemccann/dillinger/tree/master/plugins/googledrive/README.md>
   [PlOd]: <https://github.com/joemccann/dillinger/tree/master/plugins/onedrive/README.md>
   [PlMe]: <https://github.com/joemccann/dillinger/tree/master/plugins/medium/README.md>
   [PlGa]: <https://github.com/RahulHP/dillinger/blob/master/plugins/googleanalytics/README.md>
