var gulp = require('gulp');
var browserify = require('browserify');
var babelify = require('babelify');
var source = require("vinyl-source-stream");
var jsx_path = './jsx/**/*.jsx';

gulp.task('browserify', function() {
  return browserify({
      entries: ['./src/index.jsx'],
      extensions: ['.js', '.jsx']
  })
      .transform(babelify)
      .bundle()
      .pipe(source('app.js'))
      .pipe(gulp.dest('./build'));
});


gulp.task('watch', function() {
    gulp.watch(jsx_path, ['browserify']);
});
