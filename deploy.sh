docker build -t yuruh/boids:$(git describe --abbrev=0) .

# docker push yuruh/boids:$(git describe --abbrev=0)

# docker run -a stdin -a stdout --network="host" -it yuruh/boids:$(git describe --abbrev=0)

# for some reason binding -p "8080:8080" doesn't work, so I use temporarily --network="host"