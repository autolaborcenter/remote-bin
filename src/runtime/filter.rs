macro_rules! particle_filter {
    () => {
        Arc::new(Mutex::new(ParticleFilter::new(
            ParticleFilterParameters {
                incremental_timeout: Duration::from_secs(3),
                default_model: Pm1Model::default(),
                memory_rate: 0.75,
                count: 80,
                measure_weight: 8.0,
                beacon_on_robot: point(-0.2, 0.0),
                max_inconsistency: 0.2,
            },
            |model, weight| {
                Pm1Model::new(
                    model.width,
                    model.length,
                    model.wheel * (1.0 + gaussian() * (1.0 - weight) * 0.025),
                )
            },
        )))
    };
}

macro_rules! update_wheel {
    ($filter:expr) => {{
        let (sum, weight) = $filter
            .particles()
            .iter()
            .fold((0.0, 0.0), |(sum, weight), p| {
                (sum + p.model.wheel * p.weight, weight + p.weight)
            });
        let wheel = sum / weight;
        if wheel.is_normal() {
            $filter.parameters.default_model.wheel = wheel;
        }
    }};
}
