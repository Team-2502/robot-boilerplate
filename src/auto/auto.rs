use std::cell::RefCell;
use std::rc::Rc;
use serde::{Deserialize, Serialize};
use crate::Ferris;

#[derive(Serialize, Deserialize)]
pub enum Auto {
    Nothing,
}

impl Auto {
    pub fn from_dashboard(s: &str) -> Self {
        match s {
            "Nothing" => Auto::Nothing,
            _ => Auto::Nothing,
        }
    }

    pub fn name(&self) -> &'static str {
        match self {
            Auto::Nothing => "Nothing",
            _ => "none",
        }
    }

    pub fn iterator() -> Vec<Self> {
        vec![
            Auto::Nothing,
        ]
    }

    pub fn names() -> Vec<String> {
        Self::iterator()
            .iter()
            .map(|a| a.name().to_owned())
            .collect()
    }
    
    pub async fn run_auto<'a>(ferris: Rc<RefCell<Ferris>>, chosen: Auto) {
        match chosen {
            Auto::Nothing => {
                println!("No auto was selected!");
            }
        }
    }
}