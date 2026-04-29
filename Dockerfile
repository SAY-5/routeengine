# syntax=docker/dockerfile:1.7
FROM rust:1.83-slim AS build
WORKDIR /src
COPY Cargo.toml ./
COPY crates ./crates
RUN cargo build --release --bin routequery

FROM debian:bookworm-slim
RUN useradd -u 10001 -m re && apt-get update \
 && apt-get install -y --no-install-recommends ca-certificates \
 && rm -rf /var/lib/apt/lists/*
USER re
COPY --from=build /src/target/release/routequery /usr/local/bin/routequery
ENTRYPOINT ["/usr/local/bin/routequery"]
