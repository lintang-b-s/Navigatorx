# Dockerfile buat query engine
# buat deploy ke https://railway.com/ via https://hub.docker.com/r/lintangbirdas/navigatorx-cool
# harus jalanin preprocessing & customization dulu
# todo: add preprocessor & customizer Dockerfile & push image to dockerhub 

# Step 1: Modules caching
FROM golang:1.27.1-alpine3.24 AS modules
COPY go.mod go.sum /modules/
WORKDIR /modules
RUN go mod download


# Step 2: Builder
FROM golang:1.27.1-alpine3.24 AS builder
RUN apk add --no-cache gcc musl-dev
COPY --from=modules /go/pkg /go/pkg
WORKDIR /engine
COPY go.mod go.sum ./
COPY ./cmd/engine ./cmd/engine
COPY ./data/car.yaml ./data/car.yaml
COPY ./data/profiles/car ./data/profiles/car
COPY ./pkg ./pkg
RUN  go build -ldflags '-extldflags "-static"' -o /bin/engine ./cmd/engine


# Step 3: Final
FROM scratch
COPY --from=builder /bin/engine /bin/engine
COPY --from=builder /engine/data/car.yaml /data/car.yaml
COPY --from=builder /engine/data/profiles/car /data/profiles/car

CMD ["/bin/engine"]

