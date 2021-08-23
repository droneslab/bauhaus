use axiom::prelude::*;

pub struct Test {
    t: i32,
    owned_by: Option<Aid>,
}

impl Test {
    pub fn new() -> Test {
        Test {
            t: 1,
            owned_by: None,
        }
    }

    pub async fn handle(mut self, context: Context, message: Message) -> ActorResult<Self> {
        // println!{"{:?}", context};
        self.t += 1;
        println!("{}",self.t);

        Ok(Status::done(self))
    }
}
