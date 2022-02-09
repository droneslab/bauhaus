///How to add a new actor to the framework
/// * For comprehensive example refer to [`darvis::vis`](../vis/index.html)
/// 1) Create a new feature source file in src directory
/// 2) Import following crates to use actor framwork
/// ```
/// use axiom::prelude::*;
/// use serde::{Deserialize, Serialize};
/// ```
/// 3) Declare a public structure for the actor Message
/// ```
/// // Public message struct for the actor
/// #[derive(Debug, Serialize, Deserialize)]
/// pub struct VisMsg {
///    // Pose of image paths to read in/extract, Poses take 2 matrixes, pos and rot <int type, # rows, # col, data storage?>
///    new_pose: Pose,
///    // all actor ids
///    actor_ids: std::collections::HashMap<String, axiom::actors::Aid>
/// }
/// ```
/// 4) Import following crates used for Darvis framwork integration trait implementation
/// ```
/// use crate::pluginfunction::Function;
/// use crate::dvutils::*;
/// ```
/// 5) Implement new function to create an instance
/// ```
/// impl DarvisVis {
/// pub fn new(id: String) -> DarvisVis {
///    DarvisVis {
///        traj_img: Mat::new_rows_cols_with_default(750, 1000, core::CV_8UC3, core::Scalar::all(0.0)).unwrap(),
///        cam_img: Mat::default(),
///        traj_pos: DVVector3::zeros(),
///        traj_rot: DVMatrix3::zeros(),
///        id: id
///      // actor_ids: ids,
///    }
///  }
///}
///```
/// 6) Implement core atomic functionality to be called by the actor framework
/// ```
/// impl DarvisVis {
///
/// pub fn visualize(&mut self, context: Context, message: Message) -> ActorResult<()> {
///     if let Some(msg) = message.content_as::<VisMsg>() {               
/// //        ...
/// //        ...
///     }
///     Ok(Status::done(()))
/// }
/// }
/// ```
/// 7) Implement 'Function' trait for the actor. 
/// This is the trait that is called by the framwork and is the only entrypoint to your actor.
/// ```
/// impl Function for DarvisVis {
/// fn handle(&mut self, _context: axiom::prelude::Context, message: Message) -> ActorResult<()>
/// {
///     self.visualize(_context, message).unwrap();
///     Ok(Status::done(()))
/// }
/// }
/// ```
/// 
pub fn How_to_Add_a_new_Feature()
{

}


/// Once the new actor is implemented, you need to register it to the framework, in order to be used.
/// 
/// Modify the registerplugin.rs getmethod function by adding new entry in the pattern match control statement
/// ```
/// pub fn getmethod(fnname: String, id: String) -> FunctionProxy
/// {
///     match fnname.as_ref()
///     {
///  //  ...
///  //     ,
///  // Following is an example entry for the DarvisVis Function
///         "vis" => FunctionProxy {function: Box::new(crate::vis::DarvisVis::new(id))}
///  //      ,
///  //      Always add your entry Just above the _ pattern.
///         _ => FunctionProxy {function: Box::new(crate::pluginfunction::DarvisNone)}
///         ,
///     }
/// }
/// ```
///   
pub fn How_to_Register_new_Feature()
{

}


/// Type defs to be used to form an abstraction layer to the framework implementation.
/// 
/// 
/// NOTE: The reason we don't use OpenCV data structures directly, as they are not serializable and hence restrict the implementation.
/// In order to overcome the serializability, we use different library to work with algebra and to abstract the conversion, following types are defined and should be used for all Actor messages.
/// All actors should use following types to communicate messages.
/// If required, should add new types and use within the framework.
/// Please refer [`darvis::dvutil`](../dvutils/index.html) for full abstract structures type defs
/// ```
/// // Message type for this actor
/// #[derive(Debug, Serialize, Deserialize)]
/// pub struct AlignMsg {
///     // Darvis Vector of KeyPoint
///     img1_kps: DVVectorOfKeyPoint,
///     // Darvis Grayscale Matrix
///     img1_des: DVMatrixGrayscale,
///     // Darvis Vector of KeyPoint
///     img2_kps: DVVectorOfKeyPoint,
///     // Darvis Grayscale Matrix
///     img2_des: DVMatrixGrayscale,
///     // Map of actor name to actor id
///     actor_ids: std::collections::HashMap<String, axiom::actors::Aid>,
/// }
/// ```
pub fn How_to_define_Actor_Message()
{

}